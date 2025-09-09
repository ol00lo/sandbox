import asyncio
import random
import math
from dataclasses import dataclass
from enum import Enum
from typing import List, Dict
import numpy as np

@dataclass
class WorldConfig:
    WIDTH = 1350
    HEIGHT = 800
    PREY_COLOR_BGR = (255, 0, 255) #pink
    PREDATOR_COLOR_BGR = (0, 0, 0) #black
    PREY_SIZE = 12
    PREDATOR_SIZE = 25

    PREY_FOV = 60  # degrees
    PREDATOR_FOV = 30  # degrees
    PREY_DETECTION_RANGE = 200  # pixels
    PREDATOR_DETECTION_RANGE = 400  # pixels
    PREY_RESOLUTION = 40  # number of vision rays
    PREDATOR_RESOLUTION = 40  # number of vision rays

class EntityType(Enum):
    PREY = 1
    PREDATOR = 2

class Sensor:
    def __init__(self, fov, detection_range, resolution):
        self.fov = fov
        self.detection_range = detection_range
        self.resolution = resolution

        self.ray_angles = np.linspace(
            -fov / 2, fov / 2, resolution
        )

    def get_sensor_data(self, world, entity):
        distances = np.zeros(self.resolution, dtype=np.float32)
        types = np.zeros(self.resolution, dtype=np.uint8)

        for i, angle_deg in enumerate(self.ray_angles):
            angle_rad = math.radians(angle_deg + entity.direction)

            dx = math.cos(angle_rad)
            dy = math.sin(angle_rad)

            hit_distance, hit_type = world.cast_ray(
                entity.x, entity.y, dx, dy, self.detection_range, entity.id
            )

            if hit_distance < float('inf'):
                distances[i] = 1.0 / (hit_distance + 1)
                types[i] = hit_type
            else:
                distances[i] = 0.0
                types[i] = 0

        return distances, types

@dataclass
class Entity:
    id: int
    type: EntityType
    x: int
    y: int
    direction: float = 0.0
    alive: bool = True
    sensor: Sensor = None
    sensor_distances: np.ndarray = None
    sensor_types: np.ndarray = None

    def __post_init__(self):
        if self.type == EntityType.PREY:
            self.sensor = Sensor(
                WorldConfig.PREY_FOV,
                WorldConfig.PREY_DETECTION_RANGE,
                WorldConfig.PREY_RESOLUTION
            )
        elif self.type == EntityType.PREDATOR:
            self.sensor = Sensor(
                WorldConfig.PREDATOR_FOV,
                WorldConfig.PREDATOR_DETECTION_RANGE,
                WorldConfig.PREDATOR_RESOLUTION
            )

class World:
    def __init__(self):
        self.entities: Dict[int, Entity] = {}
        self.next_id = 1
        self.lock = asyncio.Lock()
    
    def add_entity(self, entity: Entity) -> None:
        self.entities[entity.id] = entity
    
    def remove_entity(self, entity_id: int) -> None:
        if entity_id in self.entities:
            del self.entities[entity_id]
    
    async def move_entity(self, entity_id, dx, dy):
        async with self.lock:
            if entity_id not in self.entities:
                return False

            entity = self.entities[entity_id]
            new_x = entity.x + dx
            new_y = entity.y + dy

            new_x %= WorldConfig.WIDTH
            new_y %= WorldConfig.HEIGHT

            entity.x = new_x
            entity.y = new_y

            if dx != 0 or dy != 0:
                entity.direction = math.degrees(math.atan2(dy, dx))
            return True

    async def get_entities_snapshot(self) -> List[Entity]:
        async with self.lock:
            entities = []
            for e in self.entities.values():
                distances, types = e.sensor.get_sensor_data(self, e)
                entity = Entity(
                    id=e.id,
                    type=e.type,
                    x=e.x,
                    y=e.y,
                    direction=e.direction,
                    alive=e.alive,
                    sensor_distances=distances,
                    sensor_types=types
                )
                entities.append(entity)
            return entities

    def cast_ray(self, start_x, start_y, dx, dy, max_distance, source_id):
        closest_distance = float('inf')
        closest_type = 0

        for entity in self.entities.values():
            if entity.id == source_id or not entity.alive:
                continue

            distance = self._ray_entity_distance(start_x, start_y, dx, dy, entity)

            if distance < closest_distance and distance <= max_distance:
                closest_distance = distance
                closest_type = entity.type.value

        return closest_distance, closest_type

    def _ray_entity_distance(self, start_x, start_y, dx, dy, entity):
        ex = entity.x - start_x
        ey = entity.y - start_y

        projection = ex * dx + ey * dy

        if projection < 0:
            return float('inf')

        closest_x = start_x + dx * projection
        closest_y = start_y + dy * projection

        distance = math.sqrt((entity.x - closest_x)**2 + (entity.y - closest_y)**2)

        entity_size = WorldConfig.PREY_SIZE if entity.type == EntityType.PREY else WorldConfig.PREDATOR_SIZE
        if distance <= entity_size:
            return projection

        return float('inf')

    async def get_sensor_data(self, entity_id):
        async with self.lock:
            if entity_id not in self.entities:
                return np.array([]), np.array([])

            entity = self.entities[entity_id]
            return entity.sensor.get_sensor_data(self, entity)

    async def find_nearby_prey(self, predator_id: int, distance: int = 10) -> List[Entity]:
        async with self.lock:
            if predator_id not in self.entities:
                return []

            predator = self.entities[predator_id]
            nearby = []

            for entity in self.entities.values():
                if entity.type != EntityType.PREY or not entity.alive:
                    continue
                dx = abs(entity.x - predator.x)
                dy = abs(entity.y - predator.y)
                dx = min(dx, WorldConfig.WIDTH - dx)
                dy = min(dy, WorldConfig.HEIGHT - dy)
                d = math.hypot(dx, dy)
                if d < distance:
                    nearby.append(entity)

            return nearby

    async def kill_entity(self, entity_id: int) -> None:
        async with self.lock:
            if entity_id in self.entities:
                self.entities[entity_id].alive = False
                self.remove_entity(entity_id)

async def prey_behavior(world: World, prey_id: int):
    while True:
        prey = world.entities.get(prey_id)
        if not prey or not prey.alive:
            break

        distances, types = await world.get_sensor_data(prey_id)

        predator_mask = (types == 2)
        if np.any(predator_mask):
            predator_angles = prey.sensor.ray_angles[predator_mask]
            sin_mean = np.mean(np.sin(np.radians(predator_angles)))
            cos_mean = np.mean(np.cos(np.radians(predator_angles)))
            avg_angle = (math.degrees(math.atan2(sin_mean, cos_mean)) + 360) % 360

            flee_angle = (avg_angle + 180) % 360
            flee_dx = int(5 * math.cos(math.radians(flee_angle)))
            flee_dy = int(5 * math.sin(math.radians(flee_angle)))

            await world.move_entity(prey_id, flee_dx, flee_dy)
        else:
            if random.random() < 0.7:
                dx = random.randint(-5, 5)
                dy = random.randint(-5, 5)
                await world.move_entity(prey_id, dx, dy)

        await asyncio.sleep(0.1)

async def predator_behavior(world: World, predator_id: int):
    while True:
        predator = world.entities.get(predator_id)
        if not predator or not predator.alive:
            break
        distances, types = await world.get_sensor_data(predator_id)

        prey_mask = (types == 1)
        if np.any(prey_mask):
            prey_angles = predator.sensor.ray_angles[prey_mask]
            sin_mean = np.mean(np.sin(np.radians(prey_angles)))
            cos_mean = np.mean(np.cos(np.radians(prey_angles)))
            avg_angle = (math.degrees(math.atan2(sin_mean, cos_mean)) + 360) % 360

            chase_dx = int(8 * math.cos(math.radians(avg_angle)))
            chase_dy = int(8 * math.sin(math.radians(avg_angle)))

            await world.move_entity(predator_id, chase_dx, chase_dy)

            nearby_prey = await world.find_nearby_prey(predator_id, 20)
            if nearby_prey:
                await world.kill_entity(nearby_prey[0].id)
        else:
            dx = random.randint(-10, 10)
            dy = random.randint(-10, 10)
            await world.move_entity(predator_id, dx, dy)

        await asyncio.sleep(0.2)

def spawn_initial_entities(world: World, n_preys=30, n_predators=5):
    for _ in range(n_preys):
        prey = Entity(
            id=world.next_id,
            type=EntityType.PREY,
            x=random.randint(0, WorldConfig.WIDTH),
            y=random.randint(0, WorldConfig.HEIGHT),
            direction=random.uniform(0, 360)
        )
        world.next_id += 1
        world.add_entity(prey)

    for _ in range(n_predators):
        predator = Entity(
            id=world.next_id,
            type=EntityType.PREDATOR,
            x=random.randint(0, WorldConfig.WIDTH),
            y=random.randint(0, WorldConfig.HEIGHT),
            direction=random.uniform(0, 360)
        )
        world.next_id += 1
        world.add_entity(predator)
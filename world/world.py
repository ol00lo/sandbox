import asyncio
import random
import math
from dataclasses import dataclass
from typing import List, Dict
import numpy as np

@dataclass
class WorldConfig:
    WIDTH = 1350
    HEIGHT = 800
    PREY_COLOR_BGR = (255, 0, 255) #pink
    PREDATOR_COLOR_BGR = (0, 0, 0) #black
    PREY_SIZE = 12
    PREDATOR_SIZE = 13

    PREY_FOV = 60  # degrees
    PREDATOR_FOV = 30  # degrees
    PREY_DETECTION_RANGE = 200  # pixels
    PREDATOR_DETECTION_RANGE = 400  # pixels
    PREY_RESOLUTION = 40  # number of vision rays
    PREDATOR_RESOLUTION = 40  # number of vision rays

class Sensor:
    def __init__(self, fov, detection_range, resolution):
        self.fov = fov
        self.detection_range = detection_range
        self.resolution = resolution

        self.ray_angles = np.linspace(
            -fov / 2, fov / 2, resolution
        )

    def get_sensor_data(self, world, entity):
        angles_rad = np.radians(self.ray_angles + entity.direction)
        dxs = np.cos(angles_rad).astype(np.float32)
        dys = np.sin(angles_rad).astype(np.float32)

        hit_distances, hit_types = world.cast_ray(
            entity.x,
            entity.y,
            dxs,
            dys,
            float(self.detection_range),
            entity.id,
        )

        distances = np.zeros(self.resolution, dtype=np.float32)
        types = hit_types.astype(np.uint8, copy=False)

        valid_mask = np.isfinite(hit_distances)
        distances[valid_mask] = 1.0 / (hit_distances[valid_mask] + 1.0)
        return distances, types

@dataclass
class Entity:
    id: int
    x: int
    y: int
    direction: float = 0.0
    alive: bool = True
    sensor: Sensor = None
    sensor_distances: np.ndarray = None
    sensor_types: np.ndarray = None

class Prey(Entity):
    def __init__(self, id, x, y, direction = 0.0, alive = True):
        super().__init__(
            id=id, x=x, y=y,
            direction=direction,
            alive=alive
        )
        self.sensor = Sensor(
            WorldConfig.PREY_FOV,
            WorldConfig.PREY_DETECTION_RANGE,
            WorldConfig.PREY_RESOLUTION
        )

class Predator(Entity):
    def __init__(self, id, x, y, direction = 0.0, alive = True):
        super().__init__(
            id=id, x=x, y=y,
            direction=direction,
            alive=alive
        )
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
                if isinstance(e, Prey):
                    entity = Prey(
                        id=e.id, x=e.x, y=e.y,
                        direction=e.direction, alive=e.alive
                    )
                else:
                    entity = Predator(
                        id=e.id, x=e.x, y=e.y,
                        direction=e.direction, alive=e.alive
                    )
                entity.sensor_distances = distances
                entity.sensor_types = types
                entities.append(entity)
            return entities

    def cast_ray(self, start_x, start_y, dxs: np.ndarray, dys: np.ndarray, max_distance: float, source_id: int):
        if dxs.ndim != 1 or dys.ndim != 1 or dxs.shape[0] != dys.shape[0]:
            raise ValueError("dxs and dys must be 1D arrays of the same length")

        num_rays = dxs.shape[0]

        if not self.entities:
            return np.full(num_rays, np.inf, dtype=np.float32), np.zeros(num_rays, dtype=np.uint8)

        entities = list(self.entities.values())

        ids = np.fromiter((e.id for e in entities), dtype=np.int64)
        alive = np.fromiter((e.alive for e in entities), dtype=bool)
        xs = np.fromiter((e.x for e in entities), dtype=np.float32)
        ys = np.fromiter((e.y for e in entities), dtype=np.float32)

        types = np.fromiter((1 if isinstance(e, Prey) else 2 for e in entities), dtype=np.uint8)
        sizes = np.where(
            types == 1,
            float(WorldConfig.PREY_SIZE),
            float(WorldConfig.PREDATOR_SIZE),
        ).astype(np.float32)

        valid_entities_mask = alive & (ids != int(source_id))
        if not np.any(valid_entities_mask):
            return np.full(num_rays, np.inf, dtype=np.float32), np.zeros(num_rays, dtype=np.uint8)

        ids = ids[valid_entities_mask]
        xs = xs[valid_entities_mask]
        ys = ys[valid_entities_mask]
        types = types[valid_entities_mask]
        sizes = sizes[valid_entities_mask]

        ex = xs - float(start_x)
        ey = ys - float(start_y)

        r2 = ex * ex + ey * ey

        projections = ex[:, None] * dxs[None, :] + ey[:, None] * dys[None, :]

        ahead_mask = projections >= 0.0

        perp2 = r2[:, None] - projections * projections
        np.maximum(perp2, 0.0, out=perp2)
        perp_dist = np.sqrt(perp2, dtype=np.float32)

        hit_mask = perp_dist <= sizes[:, None]
        within_range_mask = projections <= float(max_distance)

        valid_mask = ahead_mask & hit_mask & within_range_mask

        candidate_proj = np.where(valid_mask, projections, np.inf)

        min_indices = np.argmin(candidate_proj, axis=0)
        ray_indices = np.arange(num_rays)
        min_distances = candidate_proj[min_indices, ray_indices].astype(np.float32)

        hit_types = np.zeros(num_rays, dtype=np.uint8)
        has_hit = np.isfinite(min_distances)
        if np.any(has_hit):
            hit_types[has_hit] = types[min_indices[has_hit]]

        return min_distances, hit_types

    def _ray_entity_distance(self, start_x, start_y, dx, dy, entity):
        ex = entity.x - start_x
        ey = entity.y - start_y

        projection = ex * dx + ey * dy

        if projection < 0:
            return float('inf')

        closest_x = start_x + dx * projection
        closest_y = start_y + dy * projection

        distance = math.sqrt((entity.x - closest_x)**2 + (entity.y - closest_y)**2)

        entity_size = WorldConfig.PREY_SIZE if isinstance(entity, Prey) else WorldConfig.PREDATOR_SIZE
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
            prey_list = [e for e in self.entities.values() if isinstance(e, Prey) and e.alive]
            if not prey_list:
                return []

            prey_x = np.asarray([e.x for e in prey_list], dtype=np.float32)
            prey_y = np.asarray([e.y for e in prey_list], dtype=np.float32)

            dx = np.abs(prey_x - float(predator.x))
            dy = np.abs(prey_y - float(predator.y))
            dx = np.minimum(dx, float(WorldConfig.WIDTH) - dx)
            dy = np.minimum(dy, float(WorldConfig.HEIGHT) - dy)
            d = np.hypot(dx, dy)

            mask = d < float(distance)
            if not np.any(mask):
                return []
            return [prey_list[i] for i in np.nonzero(mask)[0].tolist()]

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
        #if np.any(predator_mask):
        if False:
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
        #if np.any(prey_mask):
        if False:
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
            nearby_prey = await world.find_nearby_prey(predator_id, 20)
            if nearby_prey:
                await world.kill_entity(nearby_prey[0].id)

        await asyncio.sleep(0.2)

def spawn_initial_entities(world: World, n_preys=30, n_predators=5):
    for _ in range(n_preys):
        prey = Prey(
            id=world.next_id,
            x=random.randint(0, WorldConfig.WIDTH),
            y=random.randint(0, WorldConfig.HEIGHT),
            direction=random.uniform(0, 360)
        )
        world.next_id += 1
        world.add_entity(prey)

    for _ in range(n_predators):
        predator = Predator(
            id=world.next_id,
            x=random.randint(0, WorldConfig.WIDTH),
            y=random.randint(0, WorldConfig.HEIGHT),
            direction=random.uniform(0, 360)
        )
        world.next_id += 1
        world.add_entity(predator)
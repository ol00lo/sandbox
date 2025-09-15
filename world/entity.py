import asyncio
import random
import math
from dataclasses import dataclass
import numpy as np
from config import WorldConfig

class Sensor:
    def __init__(self, fov, detection_range, resolution):
        self.fov = fov
        self.detection_range = detection_range
        self.resolution = resolution

        self.ray_angles = np.linspace(
            -fov / 2, fov / 2, resolution
        )

        self.hit_distances = np.full(resolution, np.inf, dtype=np.float32)
        self.hit_types = np.zeros(resolution, dtype=np.uint8)
        self.rdistances = np.zeros(resolution, dtype=np.float32)

    def scan(self, world, entity):
        angles_rad = np.radians(self.ray_angles + entity.direction)
        dxs = np.cos(angles_rad).astype(np.float32)
        dys = np.sin(angles_rad).astype(np.float32)
        self.hit_distances, self.hit_types = world.cast_ray(
            entity.x,
            entity.y,
            dxs,
            dys,
            float(self.detection_range),
            entity.id,
        )
        self.rdistances.fill(0.0)
        valid_mask = np.isfinite(self.hit_distances)
        self.rdistances[valid_mask] = 1.0 / (self.hit_distances[valid_mask] + 1.0)

@dataclass
class Entity:
    id: int
    x: int
    y: int
    direction: float = 0.0
    sensor: Sensor = None
    sensor_distances: np.ndarray = None
    sensor_types: np.ndarray = None

    async def behavior(self, world):
        while True:
            if self.id not in world.entities:
                break
            self.sensor.scan(world, self)
            self.act(world)
            await asyncio.sleep(self.delay())

    def act(self):
        raise NotImplementedError()

    def delay(self):
        raise NotImplementedError()

class Prey(Entity):
    def __init__(self, id, x, y, direction = 0.0):
        super().__init__(
            id=id, x=x, y=y,
            direction=direction
        )
        self.sensor = Sensor(
            WorldConfig.PREY_FOV,
            WorldConfig.PREY_DETECTION_RANGE,
            WorldConfig.PREY_RESOLUTION
        )

    def act(self, world):
        types = self.sensor.hit_types
        predator_mask = (types == 2)
        if np.any(predator_mask):
            predator_angles = self.sensor.ray_angles[predator_mask]
            sin_mean = np.mean(np.sin(np.radians(predator_angles)))
            cos_mean = np.mean(np.cos(np.radians(predator_angles)))
            avg_angle = (math.degrees(math.atan2(sin_mean, cos_mean)) + 360) % 360

            flee_angle = (avg_angle + 180) % 360
            flee_dx = int(5 * math.cos(math.radians(flee_angle)))
            flee_dy = int(5 * math.sin(math.radians(flee_angle)))

            world.move_entity(self.id, flee_dx, flee_dy)
        else:
            if random.random() < 0.7:
                dx = random.randint(-5, 5)
                dy = random.randint(-5, 5)
                world.move_entity(self.id, dx, dy)

    def delay(self):
        return WorldConfig.PREY_WAIT_TIME

class Predator(Entity):
    def __init__(self, id, x, y, direction = 0.0):
        super().__init__(
            id=id, x=x, y=y,
            direction=direction
        )
        self.eat_distance = 40
        self.sensor = Sensor(
            WorldConfig.PREDATOR_FOV,
            WorldConfig.PREDATOR_DETECTION_RANGE,
            WorldConfig.PREDATOR_RESOLUTION
        )

    def act(self, world):
        types = self.sensor.hit_types
        prey_mask = (types == 1)
        if np.any(prey_mask):
            prey_angles = self.sensor.ray_angles[prey_mask]
            sin_mean = np.mean(np.sin(np.radians(prey_angles)))
            cos_mean = np.mean(np.cos(np.radians(prey_angles)))
            avg_angle = (math.degrees(math.atan2(sin_mean, cos_mean)) + 360) % 360

            chase_dx = int(8 * math.cos(math.radians(avg_angle)))
            chase_dy = int(8 * math.sin(math.radians(avg_angle)))

            world.move_entity(self.id, chase_dx, chase_dy)

            nearby_prey = world.find_nearby_prey(self.id, 20)
            if nearby_prey:
                world.kill_entity(nearby_prey[0].id)
        else:
            dx = random.randint(-10, 10)
            dy = random.randint(-10, 10)
            world.move_entity(self.id, dx, dy)
            nearby_prey = world.find_nearby_prey(self.id, self.eat_distance)
            if nearby_prey:
                 world.kill_entity(nearby_prey[0].id)

    def delay(self):
        return WorldConfig.PREDATOR_WAIT_TIME

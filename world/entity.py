import asyncio
import random
import math
from dataclasses import dataclass
import numpy as np
from config import WorldConfig
from enum import Enum

class EntityType(Enum):
    NONE = 0
    PREY = 1
    PREDATOR = 2

class Sensor:
    def __init__(self, fov, detection_range, resolution):
        self.fov = fov
        self.detection_range = detection_range
        self.resolution = resolution

        self.ray_angles = np.radians(np.linspace(-fov / 2, fov / 2, resolution))

        self.hit_distances = np.full(resolution, np.inf, dtype=np.float32)
        self.hit_types = np.zeros(resolution, dtype=np.uint8)
        self.hit_ids = np.zeros(resolution, dtype=np.int64)
        self.rdistances = np.zeros(resolution, dtype=np.float32)

    def scan(self, entities, entity):
        angles_rad = self.ray_angles + entity.direction
        dxs = np.cos(angles_rad).astype(np.float32)
        dys = np.sin(angles_rad).astype(np.float32)
        self.hit_distances, self.hit_types, self.hit_ids = self.cast_ray(
            entities,
            entity.x,
            entity.y,
            dxs, dys,
            float(self.detection_range),
            entity.id,
        )
        self.rdistances.fill(0.0)
        valid_mask = np.isfinite(self.hit_distances)
        self.rdistances[valid_mask] = 1.0 / (self.hit_distances[valid_mask] + 1.0)

    def cast_ray(self, entities, start_x, start_y, dxs, dys, max_distance, source_id):
        num_rays = dxs.shape[0]

        if not entities:
            return (np.full(num_rays, np.inf, dtype=np.float32), 
                    np.zeros(num_rays, dtype=np.uint8),
                    np.zeros(num_rays, dtype=np.int64))

        entities = [e for e in entities.values() if e.id != int(source_id)]

        ids = np.fromiter((e.id for e in entities), dtype=np.int64)
        xs = np.fromiter((e.x for e in entities), dtype=np.float32)
        ys = np.fromiter((e.y for e in entities), dtype=np.float32)

        types = np.fromiter((EntityType.PREY.value if isinstance(e, Prey)
                             else EntityType.PREDATOR.value for e in entities), dtype=np.uint8)
        sizes = np.where(
            types == EntityType.PREY.value,
            float(WorldConfig.PREY_SIZE/2),
            float(WorldConfig.PREDATOR_SIZE/2),
        ).astype(np.float32)

        ex = xs - float(start_x)
        ey = ys - float(start_y)
        r2 = ex * ex + ey * ey

        projections = ex[:, None] * dxs[None, :] + ey[:, None] * dys[None, :]

        ahead_mask = projections >= 0.0

        perp2 = r2[:, None] - projections ** 2
        np.maximum(perp2, 0.0, out=perp2)
        perp_dist = np.sqrt(perp2).astype(np.float32)

        hit_mask = perp_dist <= sizes[:, None]
        within_range_mask = projections <= float(max_distance)

        valid_mask = ahead_mask & hit_mask & within_range_mask

        candidate_proj = np.where(valid_mask, projections, np.inf)

        min_indices = np.argmin(candidate_proj, axis=0)
        ray_indices = np.arange(num_rays)
        min_distances = candidate_proj[min_indices, ray_indices].astype(np.float32)

        hit_types = np.full(num_rays, EntityType.NONE.value, dtype=np.uint8)
        hit_ids = np.zeros(num_rays, dtype=np.int64)
        has_hit = np.isfinite(min_distances)
        hit_indices = np.where(has_hit)[0]
        for i in hit_indices:
            entity_idx = min_indices[i]
            hit_types[i] = types[entity_idx]
            hit_ids[i] = ids[entity_idx]

        return min_distances, hit_types, hit_ids

@dataclass
class Entity:
    id: int
    x: int
    y: int
    direction: float = 0.0 # radians
    sensor: Sensor = None

    async def behavior(self, world):
        while True:
            if self.id not in world.entities:
                break
            self.sensor.scan(world.entities, self)
            self.act(world)
            await asyncio.sleep(self.delay())

    def find_nearby_entities(self, entities, distance = float('inf'), use_sensor = True):
        if use_sensor:
            mask = self.sensor.hit_distances < float(distance)
            if not np.any(mask):
                return []

            hit_ids = self.sensor.hit_ids[mask]
            result = []
            for entity_id in set(hit_ids):
                if entity_id in entities and entity_id != self.id:
                    result.append(entities[entity_id])
            return result
        else:
            result = []
            for entity in entities.values():
                if entity.id == self.id:
                    continue
                dx = entity.x - self.x
                dy = entity.y - self.y
                dist = math.sqrt(dx*dx + dy*dy)
                if dist < distance:
                    result.append(entity)
            return result

    def find_prey_to_kill(self, entities, use_sensor = True):
        entities = self.find_nearby_entities(entities, WorldConfig.PREDATOR_EAT_DISTANCE, use_sensor)
        return [e for e in entities if isinstance(e, Prey)]

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
        predator_mask = (types == EntityType.PREDATOR.value)
        step = WorldConfig.PREY_STEP_SIZE
        if np.any(predator_mask):
            predator_distances = self.sensor.hit_distances[predator_mask]
            closest_idx = np.argmin(predator_distances)
            predator_angle = self.sensor.ray_angles[predator_mask][closest_idx]

            flee_angle = self.direction + predator_angle + math.pi
            flee_dx = int(step * math.cos(flee_angle))
            flee_dy = int(step * math.sin(flee_angle))

            world.move_entity(self.id, flee_dx, flee_dy)
        else:
            if random.random() < 0.7:
                dx = random.randint(-step, step)
                dy = random.randint(-step, step)
                world.move_entity(self.id, dx, dy)

    def delay(self):
        return WorldConfig.PREY_WAIT_TIME

class Predator(Entity):
    def __init__(self, id, x, y, direction = 0.0):
        super().__init__(
            id=id, x=x, y=y,
            direction=direction
        )
        self.sensor = Sensor(
            WorldConfig.PREDATOR_FOV,
            WorldConfig.PREDATOR_DETECTION_RANGE,
            WorldConfig.PREDATOR_RESOLUTION
        )

    def act(self, world):
        types = self.sensor.hit_types
        prey_mask = (types == EntityType.PREY.value)
        step = WorldConfig.PREDATOR_STEP_SIZE
        if np.any(prey_mask):
            closest_prey_idx = np.argmin(self.sensor.hit_distances[prey_mask])

            prey_angles = self.sensor.ray_angles[prey_mask]
            relative_angle = prey_angles[closest_prey_idx]

            absolute_angle = self.direction + relative_angle

            chase_dx = int(step * math.cos(absolute_angle))
            chase_dy = int(step * math.sin(absolute_angle))

            world.move_entity(self.id, chase_dx, chase_dy)
        else:
            dx = random.randint(-step, step)
            dy = random.randint(-step, step)
            world.move_entity(self.id, dx, dy)

        nearby_prey = self.find_prey_to_kill(world.entities)
        if nearby_prey:
             world.kill_entity(nearby_prey[0].id)

    def delay(self):
        return WorldConfig.PREDATOR_WAIT_TIME

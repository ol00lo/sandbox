import asyncio
import random
import math
from typing import List, Dict
import numpy as np
from entity import Entity, Prey, Predator
from config import WorldConfig, DrawConfig

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

    def move_entity(self, entity_id, dx, dy):
        if entity_id not in self.entities:
            return False

        entity = self.entities[entity_id]
        new_x = entity.x + dx
        new_y = entity.y + dy

        new_x %= DrawConfig.WIDTH
        new_y %= DrawConfig.HEIGHT

        entity.x = new_x
        entity.y = new_y

        if dx != 0 or dy != 0:
            entity.direction = math.degrees(math.atan2(dy, dx))
        return True

    def get_entities_snapshot(self) -> List[Entity]:
        entities = []
        for e in self.entities.values():
            e.sensor.scan(self, e)
            if isinstance(e, Prey):
                entity = Prey(
                    id=e.id, x=e.x, y=e.y,
                    direction=e.direction
                )
            else:
                entity = Predator(
                    id=e.id, x=e.x, y=e.y,
                    direction=e.direction
                )
            entity.sensor_distances = e.sensor.rdistances
            entity.sensor_types = e.sensor.hit_types
            entities.append(entity)
        return entities

    def cast_ray(self, start_x, start_y, dxs: np.ndarray, dys: np.ndarray, max_distance: float, source_id: int):
        num_rays = dxs.shape[0]

        if not self.entities:
            return np.full(num_rays, np.inf, dtype=np.float32), np.zeros(num_rays, dtype=np.uint8)

        entities = [e for e in self.entities.values() if e.id != int(source_id)]

        ids = np.fromiter((e.id for e in entities), dtype=np.int64)
        xs = np.fromiter((e.x for e in entities), dtype=np.float32)
        ys = np.fromiter((e.y for e in entities), dtype=np.float32)

        types = np.fromiter((1 if isinstance(e, Prey) else 2 for e in entities), dtype=np.uint8)
        sizes = np.where(
            types == 1,
            float(WorldConfig.PREY_SIZE/2),
            float(WorldConfig.PREDATOR_SIZE/2),
        ).astype(np.float32)

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

    async def get_sensor_data(self, entity_id):
        if entity_id not in self.entities:
            return np.array([]), np.array([])

        entity = self.entities[entity_id]
        entity.sensor.scan(self, entity)
        return entity.sensor.rdistances, entity.sensor.hit_types

    def find_nearby_prey(self, predator_id: int, distance: int = 10) -> List[Entity]:
        if predator_id not in self.entities:
            return []

        predator = self.entities[predator_id]
        predator.sensor.scan(self, predator)

        prey_mask = (predator.sensor.hit_types == 1) & (predator.sensor.hit_distances <= float(distance))
        if not np.any(prey_mask):
            return []

        prey_list = [e for e in self.entities.values() if isinstance(e, Prey)]
        if not prey_list:
            return []

        prey_x = np.asarray([e.x for e in prey_list], dtype=np.float32)
        prey_y = np.asarray([e.y for e in prey_list], dtype=np.float32)

        dx = np.abs(prey_x - float(predator.x))
        dy = np.abs(prey_y - float(predator.y))
        dx = np.minimum(dx, float(DrawConfig.WIDTH) - dx)
        dy = np.minimum(dy, float(DrawConfig.HEIGHT) - dy)
        d = np.hypot(dx, dy)

        mask = d < float(distance)
        prey_indices = np.nonzero(mask)[0]

        return [prey_list[i] for i in prey_indices.tolist()]

    def kill_entity(self, entity_id: int) -> None:
        if entity_id in self.entities:
            self.remove_entity(entity_id)

def spawn_initial_entities(world: World, n_preys=30, n_predators=5):
    for _ in range(n_preys):
        prey = Prey(
            id=world.next_id,
            x=random.randint(0, DrawConfig.WIDTH),
            y=random.randint(0, DrawConfig.HEIGHT),
            direction=random.uniform(0, 360)
        )
        world.next_id += 1
        world.add_entity(prey)

    for _ in range(n_predators):
        predator = Predator(
            id=world.next_id,
            x=random.randint(0, DrawConfig.WIDTH),
            y=random.randint(0, DrawConfig.HEIGHT),
            direction=random.uniform(0, 360)
        )
        world.next_id += 1
        world.add_entity(predator)
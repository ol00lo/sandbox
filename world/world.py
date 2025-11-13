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
            entity.direction = math.atan2(dy, dx) % (2 * math.pi)
        return True

    def get_entities_snapshot(self) -> List[Entity]:
        entities = []
        for e in self.entities.values():
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
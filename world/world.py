import asyncio
import random
import math
from dataclasses import dataclass
from enum import Enum
from typing import List, Dict

@dataclass
class WorldConfig:
    WIDTH = 1350
    HEIGHT = 800
    PREY_COLOR_BGR = (0, 0, 255) #red
    PREDATOR_COLOR_BGR = (0, 0, 0) #black
    PREY_SIZE = 12
    PREDATOR_SIZE = 20


class EntityType(Enum):
    PREY = 1
    PREDATOR = 2


@dataclass
class Entity:
    id: int
    type: EntityType
    x: int
    y: int
    alive: bool = True


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
    
    async def move_entity(self, entity_id: int, dx: int, dy: int) -> bool:
        async with self.lock:
            if entity_id not in self.entities:
                return False

            entity = self.entities[entity_id]
            new_x = entity.x + dx
            new_y = entity.y + dy

            if new_x < 0: new_x = WorldConfig.WIDTH + new_x
            elif new_x > WorldConfig.WIDTH: new_x = new_x - WorldConfig.WIDTH
            if new_y < 0: new_y = WorldConfig.HEIGHT + new_y
            elif new_y > WorldConfig.HEIGHT: new_y = new_y - WorldConfig.HEIGHT

            entity.x = new_x
            entity.y = new_y
            return True

    async def find_nearby_prey(self, predator_id: int, distance: int = 10) -> List[Entity]:
        async with self.lock:
            if predator_id not in self.entities:
                return []

            predator = self.entities[predator_id]
            nearby = []

            for entity in self.entities.values():
                if (entity.type == EntityType.PREY and 
                    entity.alive and
                    abs(entity.x - predator.x) < distance and 
                    abs(entity.y - predator.y) < distance):
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

        nearby_prey = await world.find_nearby_prey(predator_id)
        if nearby_prey:
            prey_to_eat = nearby_prey[0]
            await world.kill_entity(prey_to_eat.id)

        dx = random.randint(-10, 10)
        dy = random.randint(-10, 10)

        await world.move_entity(predator_id, dx, dy)
        await asyncio.sleep(0.2)


async def spawn_initial_entities(world: World, n_preys=30, n_predators=5):
    for _ in range(n_preys):
        prey = Entity(
            id=world.next_id,
            type=EntityType.PREY,
            x=random.randint(0, WorldConfig.WIDTH),
            y=random.randint(0, WorldConfig.HEIGHT))
        world.next_id += 1
        world.add_entity(prey)
        asyncio.create_task(prey_behavior(world, prey.id))

    for _ in range(n_predators):
        predator = Entity(
            id=world.next_id,
            type=EntityType.PREDATOR,
            x=random.randint(0, WorldConfig.WIDTH),
            y=random.randint(0, WorldConfig.HEIGHT))
        world.next_id += 1
        world.add_entity(predator)
        asyncio.create_task(predator_behavior(world, predator.id))

import asyncio
import random
from dataclasses import dataclass
from enum import Enum
from typing import List, Dict, Tuple
from flask import Flask, jsonify, render_template
import threading


app = Flask(__name__, template_folder='.')


WIDTH = 800
HEIGHT = 600
PREY_COLOR = "red"
PREDATOR_COLOR = "black"
PREY_SIZE = 5
PREDATOR_SIZE = 7


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
            new_x = max(0, min(WIDTH, entity.x + dx))
            new_y = max(0, min(HEIGHT, entity.y + dy))
            entity.x = new_x
            entity.y = new_y
            return True

    async def find_nearby_prey(self, predator_id: int, distance: int = 15) -> List[Entity]:
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


world = World()


async def prey_behavior(prey_id: int):
    while True:
        prey = world.entities.get(prey_id)
        if not prey or not prey.alive:
            break
        
        if random.random() < 0.7:
            dx = random.randint(-5, 5)
            dy = random.randint(-5, 5)
            await world.move_entity(prey_id, dx, dy)
        await asyncio.sleep(0.1)

async def predator_behavior(predator_id: int):
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


async def spawn_initial_entities(n_preys=30, n_predators=5):
    for _ in range(n_preys):
        prey = Entity(
            id=world.next_id,
            type=EntityType.PREY,
            x=random.randint(0, WIDTH),
            y=random.randint(0, HEIGHT))
        world.next_id += 1
        world.add_entity(prey)
        asyncio.create_task(prey_behavior(prey.id))
    
    for _ in range(n_predators):
        predator = Entity(
            id=world.next_id,
            type=EntityType.PREDATOR,
            x=random.randint(0, WIDTH),
            y=random.randint(0, HEIGHT))
        world.next_id += 1
        world.add_entity(predator)
        asyncio.create_task(predator_behavior(predator.id))


@app.route('/')
def index():
    return render_template('world.html',
        width=WIDTH,
        height=HEIGHT,
        prey_color=PREY_COLOR,
        predator_color=PREDATOR_COLOR,
        prey_size=PREY_SIZE,
        predator_size=PREDATOR_SIZE
    )


@app.route('/world')
def get_world():
    preys = []
    predators = []

    entities = list(world.entities.values())

    for entity in entities:
        if entity.type == EntityType.PREY:
            preys.append({'x': entity.x, 'y': entity.y})
        else:
            predators.append({'x': entity.x, 'y': entity.y})

    return jsonify({
        'preys': preys,
        'predators': predators
    })


def run_flask():
    app.run(port=5000, use_reloader=False)


async def main():
    flask_thread = threading.Thread(target=run_flask)
    flask_thread.daemon = True
    flask_thread.start()

    await spawn_initial_entities(n_preys=30, n_predators=5)

    while True:
        await asyncio.sleep(1)


if __name__ == '__main__':
    asyncio.run(main())
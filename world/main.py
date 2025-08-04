import asyncio
from flask import Flask, jsonify, render_template
import threading
from world import EntityType, World, WorldConfig, spawn_initial_entities

app = Flask(__name__, template_folder='.')

world = World()


@app.route('/')
def index():
    return render_template(
        'world.html',
        width=WorldConfig.WIDTH,
        height=WorldConfig.HEIGHT,
        prey_color=WorldConfig.PREY_COLOR,
        predator_color=WorldConfig  .PREDATOR_COLOR,
        prey_size=WorldConfig.PREY_SIZE,
        predator_size=WorldConfig.PREDATOR_SIZE
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

    await spawn_initial_entities(world)

    while True:
        await asyncio.sleep(1)


if __name__ == '__main__':
    asyncio.run(main())
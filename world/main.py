import asyncio
import threading
from world import World, Prey, Predator, spawn_initial_entities, prey_behavior, predator_behavior
from draw import render_frame, encode_png
from web_server import run_flask, set_latest_png

world = World()

async def update_frame_cache_task():
    while True:
        entities = await world.get_entities_snapshot()
        img = render_frame(entities)
        png = encode_png(img)
        set_latest_png(png)
        await asyncio.sleep(0.066)

async def run():
    entities = await world.get_entities_snapshot()
    img = render_frame(entities)
    png = encode_png(img)
    set_latest_png(png)

    for e in entities:
        if isinstance(e, Prey):
            asyncio.create_task(prey_behavior(world, e.id))
        elif isinstance(e, Predator):
            asyncio.create_task(predator_behavior(world, e.id))

    asyncio.create_task(update_frame_cache_task())

    while True:
        await asyncio.sleep(1)

def main():
    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()

    spawn_initial_entities(world)
    asyncio.run(run())

if __name__ == '__main__':
    main()
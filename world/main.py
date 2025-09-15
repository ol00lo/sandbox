import asyncio
import threading
from world import World, spawn_initial_entities
from draw import render_frame, encode_png
from web_server import run_flask, set_latest_png
from config import WorldConfig

world = World()

async def update_frame_cache_task():
    while True:
        entities = world.get_entities_snapshot()
        img = render_frame(entities, world)
        png = encode_png(img)
        set_latest_png(png)
        await asyncio.sleep((WorldConfig.UPDATE_HTML_INTERVAL - WorldConfig.DRAW_TIME)/1000)

async def run():
    entities = world.get_entities_snapshot()
    img = render_frame(entities, world)
    png = encode_png(img)
    set_latest_png(png)

    for e in entities:
        asyncio.create_task(e.behavior(world))

    asyncio.create_task(update_frame_cache_task())

    while True:
        await asyncio.sleep(1)

def main():
    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()
    spawn_initial_entities(world, 10, 10)

    asyncio.run(run())

if __name__ == '__main__':
    main()
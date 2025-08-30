import asyncio
from flask import Flask, render_template, Response
import threading
from world import World, WorldConfig, spawn_initial_entities
from draw import render_frame, encode_png

app = Flask(__name__, template_folder='.')

world = World()
event_loop = None

@app.route('/')
def index():
    return render_template('world.html',
                            width=WorldConfig.WIDTH,
                            height=WorldConfig.HEIGHT)


@app.route('/frame')
def get_frame():
    global event_loop
    if event_loop is not None:
        try:
            fut = asyncio.run_coroutine_threadsafe(world.get_entities_snapshot(), event_loop)
            entities = fut.result(timeout=0.5)
            img = render_frame(entities)
        except Exception:
            img = render_frame(list(world.entities.values()))
    else:
        img = render_frame(list(world.entities.values()))
    png = encode_png(img)
    return Response(png, mimetype='image/png')


def run_flask():
    app.run(port=5000, use_reloader=False)


async def main():
    flask_thread = threading.Thread(target=run_flask)
    flask_thread.daemon = True
    flask_thread.start()

    global event_loop
    event_loop = asyncio.get_running_loop()

    await spawn_initial_entities(world)

    while True:
        await asyncio.sleep(1)


if __name__ == '__main__':
    asyncio.run(main())
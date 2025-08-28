import asyncio
from flask import Flask, render_template, Response
import threading
from world import World, WorldConfig, spawn_initial_entities
from draw import render_frame, encode_png

app = Flask(__name__, template_folder='.')

world = World()

@app.route('/')
def index():
    return render_template('world.html',
                            width=WorldConfig.WIDTH,
                            height=WorldConfig.HEIGHT)


@app.route('/frame')
def get_frame():
    img = render_frame(world)
    png = encode_png(img)
    return Response(png, mimetype='image/png')


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
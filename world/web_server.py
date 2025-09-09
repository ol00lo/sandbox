from flask import Flask, render_template, Response
import threading
from world import WorldConfig

app = Flask(__name__, template_folder='.')

_latest_png = None
_latest_png_lock = threading.Lock()

def set_latest_png(png: bytes) -> None:
    global _latest_png
    with _latest_png_lock:
        _latest_png = png


@app.route('/')
def index():
    return render_template('world.html',
                            width=WorldConfig.WIDTH,
                            height=WorldConfig.HEIGHT)

@app.route('/frame')
def get_frame():
    with _latest_png_lock:
        png = None if _latest_png is None else _latest_png
    if png is None:
        return Response(status=503)
    return Response(png, mimetype='image/png')

def run_flask():
    app.run(port=5000, use_reloader=False)


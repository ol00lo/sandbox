import asyncio
from typing import Tuple
import cv2
import numpy as np
from world import EntityType, World, WorldConfig

def draw_heart(img, center_x, center_y, size, color_bgr):
    num_points = 100
    t = np.linspace(0, 2 * np.pi, num_points)
    x = 16 * np.sin(t) ** 3
    y = 13 * np.cos(t) - 5 * np.cos(2 * t) - 2 * np.cos(3 * t) - np.cos(4 * t)
    x = x * size / 18.0 + center_x
    y = - y * size / 18.0 + center_y
    pts = np.vstack([x, y]).T.astype(np.int32)
    cv2.fillPoly(img, [pts], color_bgr)

def draw_square(img, center_x, center_y, size, color_bgr):
    half = size // 2
    top_left = (int(center_x - half), int(center_y - half))
    bottom_right = (int(center_x + half), int(center_y + half))
    cv2.rectangle(img, top_left, bottom_right, color_bgr, thickness=-1)

def render_frame(world):
    width = WorldConfig.WIDTH
    height = WorldConfig.HEIGHT

    frame = np.full((height, width, 3), 255, dtype=np.uint8)
    cv2.rectangle(frame, (0, 0), (width - 1, height - 1), (0, 0, 0), thickness=3)

    entities = list(world.entities.values())
    for entity in entities:
        if entity.type == EntityType.PREY:
            draw_heart(frame, entity.x, entity.y,
                WorldConfig.PREY_SIZE, WorldConfig.PREY_COLOR_BGR)
        else:
            draw_square(frame, entity.x, entity.y,
                WorldConfig.PREDATOR_SIZE, WorldConfig.PREDATOR_COLOR_BGR)
    return frame

def encode_png(img):
    success, buf = cv2.imencode('.png', img)
    if not success:
        raise RuntimeError('Failed to encode PNG frame')
    return buf.tobytes()


async def render_loop(world):
    width = WorldConfig.WIDTH
    height = WorldConfig.HEIGHT

    window_name = "World"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, width, height)

    background = np.full((height, width, 3), 255, dtype=np.uint8)
    cv2.rectangle(background, (0, 0), (width - 1, height - 1), (0, 0, 0), thickness=3)

    try:
        while True:
            frame = background.copy()
            async with world.lock:
                entities = list(world.entities.values())

            for entity in entities:
                if entity.type == EntityType.PREY:
                    draw_heart(frame, entity.x, entity.y,
                        WorldConfig.PREY_SIZE, WorldConfig.PREY_COLOR_BGR)
                else:
                    draw_square(frame, entity.x, entity.y,
                        WorldConfig.PREDATOR_SIZE, WorldConfig.PREDATOR_COLOR_BGR)

            cv2.imshow(window_name, frame)
            key = cv2.waitKey(30)
            if key == 27 or key == ord('q'):
                break

            await asyncio.sleep(0.03)
    finally:
        cv2.destroyAllWindows()

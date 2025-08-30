import cv2
import numpy as np
import math
from world import EntityType, WorldConfig

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

def draw_direction_arrow(img, center_x, center_y, direction_deg, size, color_bgr):
    direction_rad = math.radians(direction_deg)

    arrow_length = size * 2
    arrow_width = size * 0.3

    tip_x = center_x + arrow_length * math.cos(direction_rad)
    tip_y = center_y + arrow_length * math.sin(direction_rad)

    base1_x = center_x + arrow_width * math.cos(direction_rad + math.pi/2)
    base1_y = center_y + arrow_width * math.sin(direction_rad + math.pi/2)
    base2_x = center_x + arrow_width * math.cos(direction_rad - math.pi/2)
    base2_y = center_y + arrow_width * math.sin(direction_rad - math.pi/2)

    arrow_points = np.array([
        [int(tip_x), int(tip_y)],
        [int(base1_x), int(base1_y)],
        [int(base2_x), int(base2_y)]
    ], dtype=np.int32)

    cv2.fillPoly(img, [arrow_points], color_bgr)

def render_frame(entities):
    width = WorldConfig.WIDTH
    height = WorldConfig.HEIGHT

    frame = np.full((height, width, 3), 255, dtype=np.uint8)
    cv2.rectangle(frame, (0, 0), (width - 1, height - 1), (0, 0, 0), thickness=3)

    for entity in entities:
        if entity.type == EntityType.PREY:
            draw_heart(frame, entity.x, entity.y,
                WorldConfig.PREY_SIZE, WorldConfig.PREY_COLOR_BGR)
            draw_direction_arrow(frame, entity.x, entity.y, entity.direction,
                WorldConfig.PREY_SIZE * 0.6, WorldConfig.PREY_COLOR_BGR)
        else:
            draw_square(frame, entity.x, entity.y,
                WorldConfig.PREDATOR_SIZE, WorldConfig.PREDATOR_COLOR_BGR)
            draw_direction_arrow(frame, entity.x, entity.y, entity.direction,
                WorldConfig.PREDATOR_SIZE * 0.6, WorldConfig.PREDATOR_COLOR_BGR)
    return frame

def encode_png(img):
    success, buf = cv2.imencode('.png', img)
    if not success:
        raise RuntimeError('Failed to encode PNG frame')
    return buf.tobytes()

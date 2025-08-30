import cv2
import numpy as np
import math
from world import EntityType, WorldConfig

def draw_prey(img, center_x, center_y, size, angle_deg, color_bgr):
    eye_radius = size
    iris_radius = eye_radius * 0.7
    pupil_radius = iris_radius * 0.5

    cv2.circle(img, (int(center_x), int(center_y)), eye_radius, (255, 255, 255), -1)
    cv2.circle(img, (int(center_x), int(center_y)), eye_radius, (0, 0, 0), 1)

    angle_rad = math.radians(angle_deg)
    max_offset = (eye_radius - iris_radius) * 0.6

    iris_x = center_x + max_offset * math.cos(angle_rad)
    iris_y = center_y - max_offset * math.sin(angle_rad)

    iris_color = color_bgr
    cv2.circle(img, (int(iris_x), int(iris_y)), int(iris_radius), iris_color, -1)
    cv2.circle(img, (int(iris_x), int(iris_y)), int(pupil_radius), (0, 0, 0), -1)

    highlight_offset = iris_radius * 0.4
    cv2.circle(img, (int(iris_x + highlight_offset), int(iris_y - highlight_offset)),
               int(iris_radius * 0.3), (255, 255, 255), -1)

    highlight2_offset = iris_radius * 0.2
    cv2.circle(img, (int(iris_x + highlight2_offset), int(iris_y - highlight2_offset * 0.5)),
               int(iris_radius * 0.15), (255, 255, 255), -1)

    eyelid_thickness = max(2, eye_radius // 10)

    top_control_points = np.array([
        [center_x - eye_radius * 1.1, center_y - eye_radius * 0.3],
        [center_x - eye_radius * 0.3, center_y - eye_radius * 0.7],
        [center_x + eye_radius * 0.3, center_y - eye_radius * 0.7],
        [center_x + eye_radius * 1.1, center_y - eye_radius * 0.3]
    ], dtype=np.float32)

    t = np.linspace(0, 1, 30)
    top_curve_points = []
    for ti in t:
        point = (1-ti)**3 * top_control_points[0] + \
                3*(1-ti)**2*ti * top_control_points[1] + \
                3*(1-ti)*ti**2 * top_control_points[2] + \
                ti**3 * top_control_points[3]
        top_curve_points.append(point.astype(np.int32))
    for i in range(len(top_curve_points) - 1):
        cv2.line(img, tuple(top_curve_points[i]), tuple(top_curve_points[i+1]),
                (0, 0, 0), eyelid_thickness)

    bottom_control_points = np.array([
        [center_x - eye_radius * 1.0, center_y + eye_radius * 0.2],
        [center_x - eye_radius * 0.2, center_y + eye_radius * 0.6],
        [center_x + eye_radius * 0.2, center_y + eye_radius * 0.6],
        [center_x + eye_radius * 1.0, center_y + eye_radius * 0.2]
    ], dtype=np.float32)

    bottom_curve_points = []
    for ti in t:
        point = (1-ti)**3 * bottom_control_points[0] + \
                3*(1-ti)**2*ti * bottom_control_points[1] + \
                3*(1-ti)*ti**2 * bottom_control_points[2] + \
                ti**3 * bottom_control_points[3]
        bottom_curve_points.append(point.astype(np.int32))

    for i in range(len(bottom_curve_points) - 1):
        cv2.line(img, tuple(bottom_curve_points[i]), tuple(bottom_curve_points[i+1]),
                (0, 0, 0), max(1, eyelid_thickness // 2))

def draw_predator(img, center_x, center_y, size, angle_deg, color_bgr):
    eye_radius = size // 2
    iris_radius = eye_radius * 0.6
    pupil_radius = iris_radius * 0.4

    cv2.circle(img, (int(center_x), int(center_y)), eye_radius, (255, 255, 255), -1)
    cv2.circle(img, (int(center_x), int(center_y)), eye_radius, (0, 0, 0), 2)

    angle_rad = math.radians(angle_deg)
    max_offset = (eye_radius - iris_radius) * 0.8

    iris_x = center_x + max_offset * math.cos(angle_rad)
    iris_y = center_y - max_offset * math.sin(angle_rad)

    iris_color = tuple(max(0, c - 40) for c in color_bgr)
    cv2.circle(img, (int(iris_x), int(iris_y)), int(iris_radius), iris_color, -1)
    cv2.circle(img, (int(iris_x), int(iris_y)), int(pupil_radius), (0, 0, 0), -1)

    highlight_offset = iris_radius * 0.3
    cv2.circle(img, (int(iris_x + highlight_offset), int(iris_y - highlight_offset)),
               int(iris_radius * 0.2), (255, 255, 255), -1)

    eyelid_thickness = max(3, eye_radius // 8)

    top_control_points = np.array([
        [center_x - eye_radius * 1.3, center_y - eye_radius * 0.4],
        [center_x - eye_radius * 0.5, center_y - eye_radius * 0.9],
        [center_x + eye_radius * 0.5, center_y - eye_radius * 0.9],
        [center_x + eye_radius * 1.3, center_y - eye_radius * 0.4]
    ], dtype=np.float32)

    t = np.linspace(0, 1, 50)
    top_curve_points = []
    for ti in t:
        point = (1-ti)**3 * top_control_points[0] + \
                3*(1-ti)**2*ti * top_control_points[1] + \
                3*(1-ti)*ti**2 * top_control_points[2] + \
                ti**3 * top_control_points[3]
        top_curve_points.append(point.astype(np.int32))

    for i in range(len(top_curve_points) - 1):
        cv2.line(img, tuple(top_curve_points[i]), tuple(top_curve_points[i+1]),
                (0, 0, 0), eyelid_thickness)

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
            draw_prey(frame, entity.x, entity.y,
                WorldConfig.PREY_SIZE, entity.direction, WorldConfig.PREY_COLOR_BGR)
        else:
            draw_predator(frame, entity.x, entity.y, WorldConfig.PREDATOR_SIZE,
                    entity.direction, WorldConfig.PREDATOR_COLOR_BGR)
    return frame

def encode_png(img):
    success, buf = cv2.imencode('.png', img)
    if not success:
        raise RuntimeError('Failed to encode PNG frame')
    return buf.tobytes()

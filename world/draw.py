import cv2
import numpy as np
import math
from world import WorldConfig, Prey, Predator

prey_eye_image = None
predator_eye_image = None
prey_pupil_image = None
predator_pupil_image = None
predator_red_pupil_image = None
predator_blue_pupil_image = None

def initialize_eye_images():
    global prey_eye_image, predator_eye_image, prey_pupil_image, predator_pupil_image, predator_red_pupil_image, predator_blue_pupil_image

    prey_eye_image = create_prey_eye_image(WorldConfig.PREY_SIZE)
    predator_eye_image = create_predator_eye_image(WorldConfig.PREDATOR_SIZE)

    prey_pupil_image = create_prey_pupil_image(WorldConfig.PREY_SIZE, WorldConfig.PREY_COLOR_BGR)
    predator_pupil_image = create_predator_pupil_image(WorldConfig.PREDATOR_SIZE, WorldConfig.PREDATOR_COLOR_BGR)

    predator_red_pupil_image = create_predator_pupil_image(WorldConfig.PREDATOR_SIZE, (255, 0, 0))
    predator_blue_pupil_image = create_predator_pupil_image(WorldConfig.PREDATOR_SIZE, (0, 0, 255))


def add_eye_to_image(img, eye_img, center_x, center_y):
    if eye_img is None:
        return

    h, w = eye_img.shape[:2]

    x_offset = int(center_x - w//2)
    y_offset = int(center_y - h//2)

    if (x_offset >= 0 and y_offset >= 0 and
        x_offset + w <= img.shape[1] and y_offset + h <= img.shape[0]):
        alpha = eye_img[:, :, 3] / 255.0
        for c in range(3):
            img[y_offset:y_offset+h, x_offset:x_offset+w, c] = (
                alpha * eye_img[:, :, c] +
                (1 - alpha) * img[y_offset:y_offset+h, x_offset:x_offset+w, c]
            )

def add_pupil_to_image(img, pupil_img, center_x, center_y, angle_deg, eye_radius):
    if pupil_img is None:
        return
    h, w = pupil_img.shape[:2]
    max_offset = eye_radius * 0.3
    angle_rad = math.radians(angle_deg)

    pupil_offset_x = max_offset * math.cos(angle_rad)
    pupil_offset_y = max_offset * math.sin(angle_rad)

    rotation_matrix = cv2.getRotationMatrix2D((w//2, h//2), angle_deg, 1.0)

    rotated_pupil = cv2.warpAffine(pupil_img, rotation_matrix, (w, h))

    x_offset = int(center_x - w//2 + pupil_offset_x)
    y_offset = int(center_y - h//2 + pupil_offset_y)

    if (x_offset >= 0 and y_offset >= 0 and
        x_offset + w <= img.shape[1] and y_offset + h <= img.shape[0]):
        alpha = rotated_pupil[:, :, 3] / 255.0
        for c in range(3):
            img[y_offset:y_offset+h, x_offset:x_offset+w, c] = (
                alpha * rotated_pupil[:, :, c] +
                (1 - alpha) * img[y_offset:y_offset+h, x_offset:x_offset+w, c]
            )


def create_prey_eye_image(size):
    center_x = size * 1.5
    center_y = size * 1.5

    eye_img = np.zeros((size * 3, size * 3, 4), dtype=np.uint8)
    cv2.circle(eye_img, (int(center_x), int(center_y)), size, (0, 0, 0, 255), 1)

    eyelid_thickness = max(2, size // 10)

    top_control_points = np.array([
        [center_x - size * 1.1, center_y - size * 0.3],
        [center_x - size * 0.3, center_y - size * 0.7],
        [center_x + size * 0.3, center_y - size * 0.7],
        [center_x + size * 1.1, center_y - size * 0.3]
    ], dtype=np.float32)

    t = np.linspace(0, 1, 30)
    top_curve_points = []
    for ti in t:
        point = (1-ti)**3 * top_control_points[0] + 3*(1-ti)**2*ti * top_control_points[1] + \
                3*(1-ti)*ti**2 * top_control_points[2] + ti**3 * top_control_points[3]
        top_curve_points.append(point.astype(np.int32))

    for i in range(len(top_curve_points) - 1):
        cv2.line(eye_img, tuple(top_curve_points[i]), tuple(top_curve_points[i+1]),
                (0, 0, 0, 255), eyelid_thickness)

    bottom_control_points = np.array([
        [center_x - size * 1.0, center_y + size * 0.1],
        [center_x - size * 0.2, center_y + size * 0.8],
        [center_x + size * 0.2, center_y + size * 0.8],
        [center_x + size * 1.0, center_y + size * 0.1]
    ], dtype=np.float32)

    bottom_curve_points = []
    for ti in t:
        point = (1-ti)**3 * bottom_control_points[0] + 3*(1-ti)**2*ti * bottom_control_points[1] + \
                3*(1-ti)*ti**2 * bottom_control_points[2] + ti**3 * bottom_control_points[3]
        bottom_curve_points.append(point.astype(np.int32))

    for i in range(len(bottom_curve_points) - 1):
        cv2.line(eye_img, tuple(bottom_curve_points[i]), tuple(bottom_curve_points[i+1]),
                (0, 0, 0, 255), max(1, eyelid_thickness // 2))

    return eye_img

def create_predator_eye_image(size):
    center_x = size * 1.5
    center_y = size * 1.5

    eye_img = np.zeros((size * 3, size * 3, 4), dtype=np.uint8)
    cv2.circle(eye_img, (int(center_x), int(center_y)), size, (0, 0, 0, 255), 2)

    eyelid_thickness = max(3, size // 8)

    top_control_points = np.array([
        [center_x - size * 1.3, center_y - size * 0.4],
        [center_x - size * 0.5, center_y - size * 0.9],
        [center_x + size * 0.5, center_y - size * 0.9],
        [center_x + size * 1.3, center_y - size * 0.4]
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
        cv2.line(eye_img, tuple(top_curve_points[i]), tuple(top_curve_points[i+1]),
                (0, 0, 0, 255), eyelid_thickness)

    return eye_img


def create_prey_pupil_image(size, color_bgr):
    iris_radius = size * 0.7
    pupil_radius = iris_radius * 0.5

    pupil_img = np.zeros((size * 3, size * 3, 4), dtype=np.uint8)

    center_x = size * 1.5
    center_y = size * 1.5

    iris_color = (*color_bgr, 255)
    cv2.circle(pupil_img, (int(center_x), int(center_y)), int(iris_radius), iris_color, -1)
    cv2.circle(pupil_img, (int(center_x), int(center_y)), int(pupil_radius), (0, 0, 0, 255), -1)

    highlight_offset = iris_radius * 0.4
    cv2.circle(pupil_img, (int(center_x + highlight_offset), int(center_y + highlight_offset)),
               int(iris_radius * 0.3), (255, 255, 255, 255), -1)

    highlight2_offset = iris_radius * 0.2
    cv2.circle(pupil_img, (int(center_x + highlight2_offset), int(center_y + highlight2_offset * 0.5)),
               int(iris_radius * 0.15), (255, 255, 255, 255), -1)

    return pupil_img

def create_predator_pupil_image(size, color_bgr):
    iris_radius = size * 0.6
    pupil_radius = iris_radius * 0.4

    pupil_img = np.zeros((size * 3, size * 3, 4), dtype=np.uint8)

    center_x = size * 1.5
    center_y = size * 1.5

    iris_color = tuple(max(0, c - 40) for c in color_bgr) + (255,)
    cv2.circle(pupil_img, (int(center_x), int(center_y)), int(iris_radius), iris_color, -1)
    cv2.circle(pupil_img, (int(center_x), int(center_y)), int(pupil_radius), (0, 0, 0, 255), -1)

    highlight_offset = iris_radius * 0.3
    cv2.circle(pupil_img, (int(center_x + highlight_offset), int(center_y + highlight_offset)),
               int(iris_radius * 0.2), (255, 255, 255, 255), -1)

    return pupil_img


def draw_sensor(img, center_x, center_y, direction_deg, fov_deg, range_px, color_bgr, thickness=1):
    start_deg = direction_deg - fov_deg / 2.0
    end_deg = direction_deg + fov_deg / 2.0

    start_rad = math.radians(start_deg)
    end_rad = math.radians(end_deg)

    sx = int(center_x + range_px * math.cos(start_rad))
    sy = int(center_y + range_px * math.sin(start_rad))
    ex = int(center_x + range_px * math.cos(end_rad))
    ey = int(center_y + range_px * math.sin(end_rad))

    cv2.line(img, (int(center_x), int(center_y)), (sx, sy), color_bgr, thickness)
    cv2.line(img, (int(center_x), int(center_y)), (ex, ey), color_bgr, thickness)
    cv2.ellipse(img, (int(center_x), int(center_y)), (int(range_px), int(range_px)), 0,
                start_deg, end_deg, color_bgr, thickness)

def render_frame(entities):
    width = WorldConfig.WIDTH
    height = WorldConfig.HEIGHT

    frame = np.full((height, width, 3), 255, dtype=np.uint8)
    cv2.rectangle(frame, (0, 0), (width - 1, height - 1), (0, 0, 0), thickness=3)

    global prey_eye_image, predator_eye_image,\
            prey_pupil_image, predator_pupil_image,\
             predator_blue_pupil_image, predator_red_pupil_image

    if prey_eye_image is None or predator_eye_image is None:
        initialize_eye_images()

    for entity in entities:
        if isinstance(entity, Prey):
            add_pupil_to_image(frame, prey_pupil_image, entity.x, entity.y, 
                               entity.direction, WorldConfig.PREY_SIZE)
            add_eye_to_image(frame, prey_eye_image, entity.x, entity.y)
        elif isinstance(entity, Predator):
            pupil = predator_pupil_image
            if entity.sensor_distances is not None and entity.sensor_types is not None:
                valid_mask = entity.sensor_distances > 0
                if np.any(valid_mask):
                    closest_idx = np.argmax(entity.sensor_distances[valid_mask])
                    closest_type = entity.sensor_types[valid_mask][closest_idx]
                    if closest_type == 1:
                        pupil = predator_blue_pupil_image
                    elif closest_type == 2:
                        pupil = predator_red_pupil_image
            add_pupil_to_image(frame, pupil, entity.x, entity.y,
                               entity.direction, WorldConfig.PREDATOR_SIZE // 2)
            add_eye_to_image(frame, predator_eye_image, entity.x, entity.y)
            draw_sensor(frame, entity.x, entity.y, entity.direction, WorldConfig.PREDATOR_FOV,
                                WorldConfig.PREDATOR_DETECTION_RANGE, (0, 200, 0), 1)
        else: raise NotImplementedError("...")
    return frame

def encode_png(img):
    success, buf = cv2.imencode('.png', img)
    if not success:
        raise RuntimeError('Failed to encode PNG frame')
    return buf.tobytes()

import cv2
import numpy as np
import math
from world import Prey, Predator
from config import DrawConfig, WorldConfig

class BasicImages:
    def __init__(self):
        self.prey_eye_image = self._create_prey_eye_image(WorldConfig.PREY_SIZE)
        self.predator_eye_image = self._create_predator_eye_image(WorldConfig.PREDATOR_SIZE)
        self.prey_pupil_image = self._create_prey_pupil_image(WorldConfig.PREY_SIZE, DrawConfig.PREY_COLOR_BGR)
        self.predator_pupil_image = self._create_predator_pupil_image(WorldConfig.PREDATOR_SIZE, DrawConfig.PREDATOR_COLOR_BGR)
        self.predator_red_pupil_image = self._create_predator_pupil_image(WorldConfig.PREDATOR_SIZE, (0, 0, 255))
        self.predator_blue_pupil_image = self._create_predator_pupil_image(WorldConfig.PREDATOR_SIZE, (255, 0, 0))

    def _create_prey_eye_image(self, size):
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

    def _create_predator_eye_image(self, size):
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

    def _create_prey_pupil_image(self, size, color_bgr):
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

    def _create_predator_pupil_image(self, size, color_bgr):
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


_basic_images = BasicImages()



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

def add_pupil_to_image(img, pupil_img, center_x, center_y, angle_rad, eye_radius):
    if pupil_img is None:
        return
    h, w = pupil_img.shape[:2]
    max_offset = eye_radius * 0.3

    pupil_offset_x = max_offset * math.cos(angle_rad)
    pupil_offset_y = max_offset * math.sin(angle_rad)

    angle_deg = math.degrees(angle_rad)
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

def draw_sensor(img, center_x, center_y, direction_rad, fov_deg, range_px, color_bgr, thickness=1):
    fov_rad = math.radians(fov_deg)
    start_rad = direction_rad - fov_rad / 2.0
    end_rad = direction_rad + fov_rad / 2.0

    sx = int(center_x + range_px * math.cos(start_rad))
    sy = int(center_y + range_px * math.sin(start_rad))
    ex = int(center_x + range_px * math.cos(end_rad))
    ey = int(center_y + range_px * math.sin(end_rad))

    cv2.line(img, (int(center_x), int(center_y)), (sx, sy), color_bgr, thickness)
    cv2.line(img, (int(center_x), int(center_y)), (ex, ey), color_bgr, thickness)

    start_deg = math.degrees(start_rad)
    end_deg = math.degrees(end_rad)
    cv2.ellipse(img, (int(center_x), int(center_y)), (int(range_px), int(range_px)), 0,
                start_deg, end_deg, color_bgr, thickness)

def draw_entity_id(img, entity_id, center_x, center_y):
    text = str(entity_id)

    color = (0, 0, 0)
    text_scale = 0.5
    text_thickness = 1

    text_x = int(center_x + 15)
    text_y = int(center_y + 15)

    cv2.putText(img, text, (text_x, text_y),
                cv2.FONT_HERSHEY_SIMPLEX, text_scale, color, text_thickness)

def render_frame(entities, world):
    width = DrawConfig.WIDTH
    height = DrawConfig.HEIGHT

    frame = np.full((height, width, 3), 255, dtype=np.uint8)
    cv2.rectangle(frame, (0, 0), (width - 1, height - 1), (0, 0, 0), thickness=3)

    for entity in entities:
        entity.sensor.scan(world.entities, entity)
        if isinstance(entity, Prey):
            add_pupil_to_image(frame, _basic_images.prey_pupil_image, entity.x, entity.y, 
                               entity.direction, WorldConfig.PREY_SIZE)
            draw_entity_id(frame, entity.id, entity.x, entity.y)
            add_eye_to_image(frame, _basic_images.prey_eye_image, entity.x, entity.y)
            # draw_sensor(frame, entity.x, entity.y, entity.direction, WorldConfig.PREY_FOV,
            #                    WorldConfig.PREY_DETECTION_RANGE, (150, 0, 150), 1)
        elif isinstance(entity, Predator):
            pupil = _basic_images.predator_pupil_image
            nearby_preys = entity.find_nearby_entities(world.entities, WorldConfig.PREDATOR_DETECTION_RANGE)
            if nearby_preys:
                pupil = (_basic_images.predator_blue_pupil_image if isinstance(nearby_preys[0], Predator)
                            else _basic_images.predator_red_pupil_image)

            add_pupil_to_image(frame, pupil, entity.x, entity.y,
                               entity.direction, WorldConfig.PREDATOR_SIZE // 2)
            draw_entity_id(frame, entity.id, entity.x, entity.y)
            add_eye_to_image(frame, _basic_images.predator_eye_image, entity.x, entity.y)
            draw_sensor(frame, entity.x, entity.y, entity.direction, WorldConfig.PREDATOR_FOV,
                                WorldConfig.PREDATOR_DETECTION_RANGE, (0, 200, 0), 1)
        else: raise NotImplementedError(f"uknown entity type: {type(entity).name}")
    return frame

def encode_png(img):
    success, buf = cv2.imencode('.png', img)
    if not success:
        raise RuntimeError('Failed to encode PNG frame')
    return buf.tobytes()
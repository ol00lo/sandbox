from dataclasses import dataclass

@dataclass
class DrawConfig:
    WIDTH = 1350
    HEIGHT = 800
    PREY_COLOR_BGR = (255, 0, 255) #pink
    PREDATOR_COLOR_BGR = (0, 0, 0) #black

@dataclass
class WorldConfig:
    PREY_SIZE = 12
    PREDATOR_SIZE = 13

    PREY_FOV = 60  # degrees
    PREDATOR_FOV = 30  # degrees
    PREY_DETECTION_RANGE = 200  # pixels
    PREDATOR_DETECTION_RANGE = 400  # pixels
    PREY_RESOLUTION = 40  # number of vision rays
    PREDATOR_RESOLUTION = 40  # number of vision rays

    PREY_WAIT_TIME = 0.1
    PREDATOR_WAIT_TIME = 0.2
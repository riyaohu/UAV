"""
Configuration file for UAV Search Simulator
"""
import os

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# Window settings
WINDOW_WIDTH = 1200
WINDOW_HEIGHT = 800
FPS = 60
WINDOW_TITLE = "UAV Search Algorithm Simulator"

# Colors (R, G, B)
COLOR_WHITE = (255, 255, 255)
COLOR_BLACK = (0, 0, 0)
COLOR_RED = (255, 0, 0)
COLOR_GREEN = (0, 255, 0)
COLOR_BLUE = (0, 0, 255)
COLOR_YELLOW = (255, 255, 0)
COLOR_GRAY = (128, 128, 128)
COLOR_DETECTION_RANGE = (0, 255, 0, 50)  # Green with transparency

# UAV settings
UAV_SPEED = 3.0  # pixels per frame
UAV_SIZE = 40  # display size in pixels
UAV_START_X = 100  # starting position x
UAV_START_Y = 100  # starting position y
UAV_DETECTION_RADIUS = 80  # detection range in pixels

# Target settings
TARGET_SIZE = 30  # display size in pixels
TARGET_X = 600  # target position x (can be random)
TARGET_Y = 400  # target position y (can be random)

# Map settings
MAP_IMAGE_PATH = os.path.join(BASE_DIR, "img", "map_1.png")
UAV_IMAGE_PATH = os.path.join(BASE_DIR, "img", "uav.png")
TARGET_IMAGE_PATH = os.path.join(BASE_DIR, "img", "target.png")


# Algorithm settings
RANDOM_SEARCH_TURN_PROBABILITY = 0.05  # probability to change direction
RANDOM_SEARCH_MAX_TURN_ANGLE = 45  # max turn angle in degrees

# Simulation settings
SHOW_DETECTION_RANGE = True  # show detection circle
SHOW_TRAJECTORY = True  # show UAV path
TRAJECTORY_COLOR = COLOR_BLUE
TRAJECTORY_WIDTH = 2
MAX_TRAJECTORY_POINTS = 500  # limit trajectory history

# UI settings
INFO_PANEL_X = 10
INFO_PANEL_Y = 10
INFO_FONT_SIZE = 24
INFO_TEXT_COLOR = COLOR_BLACK
INFO_BG_COLOR = (255, 255, 255, 200)  # white with transparency

#自动终止条件
MAX_FRAMES = 5000          # 最大允许运行帧数
MAX_DISTANCE = 20000.0    # 最大允许飞行距离（可选）
STOP_WHEN_ALL_FOUND = True

##复现性
DEFAULT_SEED = 201  # 用于 demo/main 的固定随机种子


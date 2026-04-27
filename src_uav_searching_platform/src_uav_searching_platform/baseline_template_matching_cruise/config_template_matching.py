"""
Independent configuration for Template Matching + Cruise baseline.
"""

import os


BASELINE_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(BASELINE_DIR)

# Window settings
WINDOW_WIDTH = 1200
WINDOW_HEIGHT = 800
FPS = 60
WINDOW_TITLE = "无人机搜索算法仿真平台 - 模板匹配引导搜索基线"

# Asset paths
MAP_IMAGE_PATH = os.path.join(PROJECT_DIR, "img", "map_1.png")
UAV_IMAGE_PATH = os.path.join(PROJECT_DIR, "img", "uav.png")
TARGET_TEMPLATE_PATH = os.path.join(PROJECT_DIR, "img", "target.png")

# UAV and target settings
UAV_SPEED = 3.0
UAV_SIZE = 40
UAV_START_X = 100
UAV_START_Y = 100
UAV_DETECTION_RADIUS = 80

TARGET_SIZE = 30
TARGET_X = 600
TARGET_Y = 400
RANDOM_TARGET = False
TARGET_SAFE_MARGIN = 60

# Search mode thresholds
MATCH_SCORE_HIGH = 0.62
MATCH_SCORE_LOW = 0.45
SCORE_SMOOTH_ALPHA = 0.40

# Template matching region settings
ROI_WIDTH = 360
ROI_HEIGHT = 260
ROI_FORWARD_OFFSET = 90
TEMPLATE_SCALES = (0.80, 1.00, 1.20, 1.35)

# Motion constraints
MAX_TURN_GUIDE_DEG = 14.0
MAX_TURN_VERIFY_DEG = 6.0
MAX_TURN_CRUISE_DEG = 10.0

# Cruise path settings
SWEEP_MARGIN = 50
LANE_SPACING = 120
WAYPOINT_REACH_DISTANCE = 24

# Simulation controls
MAX_FRAMES = 20000
SHOW_DETECTION_RANGE = True
SHOW_TRAJECTORY = True
MAX_TRAJECTORY_POINTS = 1000

# Logging and output
PRINT_LOG_INTERVAL = 15
SAVE_IMAGE_INTERVAL = 30
RESULT_DIR = os.path.join(BASELINE_DIR, "results")
IMAGE_OUTPUT_SUBDIR = "标注图像"
CSV_LOG_NAME = "运行日志.csv"

# UI settings
FONT_NAME = "Microsoft YaHei"
INFO_FONT_SIZE = 22
INFO_PANEL_X = 10
INFO_PANEL_Y = 10
INFO_BG_COLOR = (255, 255, 255, 210)
INFO_TEXT_COLOR = (0, 0, 0)

# Colors
COLOR_WHITE = (255, 255, 255)
COLOR_BLACK = (0, 0, 0)
COLOR_RED = (255, 0, 0)
COLOR_GREEN = (0, 255, 0)
COLOR_BLUE = (0, 0, 255)
COLOR_YELLOW = (255, 255, 0)
COLOR_CYAN = (0, 255, 255)
COLOR_DETECTION_RANGE = (0, 255, 0, 50)
TRAJECTORY_COLOR = COLOR_BLUE
TRAJECTORY_WIDTH = 2

# Algorithm label
ALGORITHM_NAME = "模板匹配引导搜索（Template Matching + 巡航）"

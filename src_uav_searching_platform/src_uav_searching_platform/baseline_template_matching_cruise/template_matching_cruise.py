"""
Template Matching + Cruise baseline algorithm.
"""

import math
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import pygame

try:
    import cv2
except ImportError as exc:  # pragma: no cover
    raise ImportError(
        "未安装 opencv-python。请先安装: pip install opencv-python"
    ) from exc


class TemplateMatchingCruise:
    """Template matching guided search with cruise fallback."""

    def __init__(self, map_width: int, map_height: int, cfg: Any):
        self.map_width = map_width
        self.map_height = map_height
        self.cfg = cfg
        self.name = cfg.ALGORITHM_NAME

        self.mode = "巡航模式"
        self.current_heading = 0.0
        self.score_smooth = 0.0
        self.frame_index = 0
        self.waypoint_index = 0
        self.last_info: Dict[str, Any] = {}

        self.waypoints = self._build_snake_waypoints()
        self.templates = self._build_templates()

    def reset(self, start_x: float, start_y: float, start_heading: float = 0.0) -> None:
        self.mode = "巡航模式"
        self.current_heading = start_heading % 360.0
        self.score_smooth = 0.0
        self.frame_index = 0
        self.waypoint_index = self._nearest_waypoint_index(start_x, start_y)
        self.last_info = {}

    def get_name(self) -> str:
        return self.name

    def get_debug_info(self) -> Dict[str, Any]:
        return dict(self.last_info)

    def get_next_position(
        self,
        current_x: float,
        current_y: float,
        current_angle: float,
        **kwargs: Any,
    ) -> Tuple[float, float]:
        scene_surface = kwargs.get("scene_surface")
        if scene_surface is None:
            raise ValueError("TemplateMatchingCruise 需要 scene_surface 参数")

        if self.frame_index == 0:
            self.current_heading = current_angle % 360.0

        gray = self._surface_to_gray(scene_surface)
        roi_gray, roi_rect = self._extract_forward_roi(gray, current_x, current_y, self.current_heading)
        score_now, peak = self._match_template(roi_gray, roi_rect)

        if self.frame_index == 0:
            self.score_smooth = score_now
        else:
            alpha = self.cfg.SCORE_SMOOTH_ALPHA
            self.score_smooth = alpha * score_now + (1.0 - alpha) * self.score_smooth

        if peak is not None and self.score_smooth >= self.cfg.MATCH_SCORE_HIGH:
            self.mode = "视觉引导"
            desired_heading = self._angle_to(current_x, current_y, peak[0], peak[1])
            max_turn = self.cfg.MAX_TURN_GUIDE_DEG
        elif peak is not None and self.score_smooth >= self.cfg.MATCH_SCORE_LOW:
            self.mode = "候选验证"
            desired_heading = self._angle_to(current_x, current_y, peak[0], peak[1])
            max_turn = self.cfg.MAX_TURN_VERIFY_DEG
        else:
            self.mode = "巡航模式"
            desired_heading = self._cruise_heading(current_x, current_y)
            max_turn = self.cfg.MAX_TURN_CRUISE_DEG

        self.current_heading = self._limit_turn(self.current_heading, desired_heading, max_turn)
        next_x, next_y = self._step_forward(current_x, current_y, self.current_heading, self.cfg.UAV_SPEED)
        next_x, next_y, hit_boundary = self._apply_boundary_rule(next_x, next_y)

        if hit_boundary:
            self.mode = "巡航模式"
            self.score_smooth = min(self.score_smooth, self.cfg.MATCH_SCORE_LOW * 0.90)

        self.last_info = {
            "帧号": self.frame_index,
            "模式": self.mode,
            "匹配分数": float(score_now),
            "平滑分数": float(self.score_smooth),
            "当前位置": (float(current_x), float(current_y)),
            "下一位置": (float(next_x), float(next_y)),
            "当前航向": float(self.current_heading),
            "目标是否锁定": "是" if self.score_smooth >= self.cfg.MATCH_SCORE_HIGH else "否",
            "峰值坐标": peak,
            "ROI矩形": roi_rect,
            "边界触发": "是" if hit_boundary else "否",
        }
        self.frame_index += 1
        return next_x, next_y

    def _build_snake_waypoints(self) -> List[Tuple[float, float]]:
        margin = self.cfg.SWEEP_MARGIN
        lane_spacing = self.cfg.LANE_SPACING
        left = float(margin)
        right = float(self.map_width - margin)
        y = float(margin)
        go_right = True
        waypoints: List[Tuple[float, float]] = []

        while y <= self.map_height - margin:
            x_line = right if go_right else left
            waypoints.append((x_line, y))
            next_y = y + lane_spacing
            if next_y <= self.map_height - margin:
                waypoints.append((x_line, next_y))
            y = next_y
            go_right = not go_right

        if not waypoints:
            waypoints.append((self.map_width * 0.5, self.map_height * 0.5))
        return waypoints

    def _build_templates(self) -> List[np.ndarray]:
        template_color = self._cv2_imread_unicode(self.cfg.TARGET_TEMPLATE_PATH, cv2.IMREAD_COLOR)
        if template_color is None:
            raise FileNotFoundError(f"无法加载目标模板: {self.cfg.TARGET_TEMPLATE_PATH}")

        template_gray = cv2.cvtColor(template_color, cv2.COLOR_BGR2GRAY)
        template_gray = self._preprocess_gray(template_gray)

        templates: List[np.ndarray] = []
        for scale in self.cfg.TEMPLATE_SCALES:
            width = max(8, int(template_gray.shape[1] * scale))
            height = max(8, int(template_gray.shape[0] * scale))
            interpolation = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
            resized = cv2.resize(template_gray, (width, height), interpolation=interpolation)
            templates.append(resized)
        return templates

    @staticmethod
    def _cv2_imread_unicode(path: str, flags: int) -> Optional[np.ndarray]:
        try:
            binary = np.fromfile(path, dtype=np.uint8)
            if binary.size == 0:
                return None
            return cv2.imdecode(binary, flags)
        except OSError:
            return None

    @staticmethod
    def _preprocess_gray(gray: np.ndarray) -> np.ndarray:
        normalized = cv2.equalizeHist(gray)
        return cv2.GaussianBlur(normalized, (3, 3), 0)

    def _surface_to_gray(self, surface: pygame.Surface) -> np.ndarray:
        rgb = pygame.surfarray.array3d(surface)
        rgb = np.transpose(rgb, (1, 0, 2))
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        return self._preprocess_gray(gray)

    def _extract_forward_roi(
        self,
        gray: np.ndarray,
        current_x: float,
        current_y: float,
        heading_deg: float,
    ) -> Tuple[np.ndarray, Tuple[int, int, int, int]]:
        half_w = self.cfg.ROI_WIDTH // 2
        half_h = self.cfg.ROI_HEIGHT // 2
        rad = math.radians(heading_deg)
        center_x = current_x + math.cos(rad) * self.cfg.ROI_FORWARD_OFFSET
        center_y = current_y + math.sin(rad) * self.cfg.ROI_FORWARD_OFFSET

        x1 = max(0, int(center_x - half_w))
        y1 = max(0, int(center_y - half_h))
        x2 = min(self.map_width, int(center_x + half_w))
        y2 = min(self.map_height, int(center_y + half_h))

        if x2 - x1 < 20 or y2 - y1 < 20:
            x1 = max(0, int(current_x - half_w))
            y1 = max(0, int(current_y - half_h))
            x2 = min(self.map_width, int(current_x + half_w))
            y2 = min(self.map_height, int(current_y + half_h))

        roi = gray[y1:y2, x1:x2]
        return roi, (x1, y1, x2, y2)

    def _match_template(
        self,
        roi_gray: np.ndarray,
        roi_rect: Tuple[int, int, int, int],
    ) -> Tuple[float, Optional[Tuple[float, float]]]:
        if roi_gray.size == 0:
            return 0.0, None

        best_score = 0.0
        best_peak = None

        for template in self.templates:
            t_height, t_width = template.shape[:2]
            r_height, r_width = roi_gray.shape[:2]
            if t_height > r_height or t_width > r_width:
                continue

            response = cv2.matchTemplate(roi_gray, template, cv2.TM_CCOEFF_NORMED)
            _, max_val, _, max_loc = cv2.minMaxLoc(response)
            if max_val >= best_score:
                peak_x = roi_rect[0] + max_loc[0] + t_width * 0.5
                peak_y = roi_rect[1] + max_loc[1] + t_height * 0.5
                best_score = float(max_val)
                best_peak = (float(peak_x), float(peak_y))

        return best_score, best_peak

    def _nearest_waypoint_index(self, x: float, y: float) -> int:
        best_index = 0
        best_dist = float("inf")
        for index, waypoint in enumerate(self.waypoints):
            dist = (waypoint[0] - x) ** 2 + (waypoint[1] - y) ** 2
            if dist < best_dist:
                best_dist = dist
                best_index = index
        return best_index

    def _cruise_heading(self, current_x: float, current_y: float) -> float:
        if not self.waypoints:
            return self.current_heading

        waypoint = self.waypoints[self.waypoint_index]
        if self._distance(current_x, current_y, waypoint[0], waypoint[1]) < self.cfg.WAYPOINT_REACH_DISTANCE:
            self.waypoint_index = (self.waypoint_index + 1) % len(self.waypoints)
            waypoint = self.waypoints[self.waypoint_index]
        return self._angle_to(current_x, current_y, waypoint[0], waypoint[1])

    @staticmethod
    def _distance(x1: float, y1: float, x2: float, y2: float) -> float:
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    @staticmethod
    def _angle_to(x1: float, y1: float, x2: float, y2: float) -> float:
        return math.degrees(math.atan2(y2 - y1, x2 - x1)) % 360.0

    @staticmethod
    def _normalize_signed(angle_deg: float) -> float:
        return (angle_deg + 180.0) % 360.0 - 180.0

    def _limit_turn(self, current: float, target: float, max_turn: float) -> float:
        delta = self._normalize_signed(target - current)
        delta = max(-max_turn, min(max_turn, delta))
        return (current + delta) % 360.0

    @staticmethod
    def _step_forward(x: float, y: float, heading_deg: float, speed: float) -> Tuple[float, float]:
        rad = math.radians(heading_deg)
        return x + speed * math.cos(rad), y + speed * math.sin(rad)

    def _apply_boundary_rule(self, x: float, y: float) -> Tuple[float, float, bool]:
        hit = False
        min_x, max_x = 0.0, float(self.map_width)
        min_y, max_y = 0.0, float(self.map_height)

        x_clamped = max(min_x, min(max_x, x))
        y_clamped = max(min_y, min(max_y, y))
        if x_clamped != x or y_clamped != y:
            hit = True
            rad = math.radians(self.current_heading)
            x_dir = math.cos(rad)
            y_dir = math.sin(rad)
            if x_clamped != x:
                self.current_heading = 90.0 if y_dir >= 0 else 270.0
            if y_clamped != y:
                self.current_heading = 0.0 if x_dir >= 0 else 180.0

        return x_clamped, y_clamped, hit

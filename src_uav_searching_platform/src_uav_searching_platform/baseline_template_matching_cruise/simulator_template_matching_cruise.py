"""
Independent simulator for Template Matching + Cruise baseline.
"""

import csv
import math
import os
import random
from datetime import datetime
from typing import Optional, Tuple

import pygame

from . import config_template_matching as cfg
from .template_matching_cruise import TemplateMatchingCruise


class TemplateMatchingCruiseSimulator:
    """Standalone simulator for baseline idea 1."""

    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((cfg.WINDOW_WIDTH, cfg.WINDOW_HEIGHT))
        pygame.display.set_caption(cfg.WINDOW_TITLE)
        self.clock = pygame.time.Clock()

        self.font = pygame.font.SysFont(cfg.FONT_NAME, cfg.INFO_FONT_SIZE)
        if self.font is None:
            self.font = pygame.font.Font(None, cfg.INFO_FONT_SIZE)

        self.map_surface = self._load_map(cfg.MAP_IMAGE_PATH, cfg.WINDOW_WIDTH, cfg.WINDOW_HEIGHT)
        self.uav_image = self._load_image(cfg.UAV_IMAGE_PATH, cfg.UAV_SIZE)
        self.target_image = self._load_image(cfg.TARGET_TEMPLATE_PATH, cfg.TARGET_SIZE)

        self.algorithm = TemplateMatchingCruise(cfg.WINDOW_WIDTH, cfg.WINDOW_HEIGHT, cfg)
        self.running = True
        self.paused = False
        self.search_complete = False

        self._prepare_output()
        self.reset()

    def _prepare_output(self) -> None:
        os.makedirs(cfg.RESULT_DIR, exist_ok=True)
        run_stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_dir = os.path.join(cfg.RESULT_DIR, run_stamp)
        self.image_dir = os.path.join(self.run_dir, cfg.IMAGE_OUTPUT_SUBDIR)
        os.makedirs(self.image_dir, exist_ok=True)

        self.csv_path = os.path.join(self.run_dir, cfg.CSV_LOG_NAME)
        self.csv_file = open(self.csv_path, "w", encoding="utf-8-sig", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(
            ["时间帧", "模式", "匹配分数", "平滑分数", "x坐标", "y坐标", "目标是否锁定", "是否发现目标"]
        )

        print("输出目录:", self.run_dir)
        print("日志文件:", self.csv_path)

    def _load_map(self, path: str, width: int, height: int) -> pygame.Surface:
        image = pygame.image.load(path).convert()
        return pygame.transform.scale(image, (width, height))

    def _load_image(self, path: str, size: int) -> pygame.Surface:
        image = pygame.image.load(path).convert_alpha()
        return pygame.transform.scale(image, (size, size))

    def _choose_target_position(self) -> Tuple[float, float]:
        if not cfg.RANDOM_TARGET:
            return float(cfg.TARGET_X), float(cfg.TARGET_Y)

        margin = cfg.TARGET_SAFE_MARGIN
        x = random.randint(margin, cfg.WINDOW_WIDTH - margin)
        y = random.randint(margin, cfg.WINDOW_HEIGHT - margin)
        return float(x), float(y)

    def reset(self) -> None:
        self.uav_x = float(cfg.UAV_START_X)
        self.uav_y = float(cfg.UAV_START_Y)
        self.uav_heading = 0.0
        self.distance_traveled = 0.0
        self.frames = 0
        self.trajectory = [(self.uav_x, self.uav_y)]

        self.target_x, self.target_y = self._choose_target_position()
        self.target_found = False
        self.search_complete = False
        self.paused = False
        self.last_debug = {}

        self.algorithm.reset(self.uav_x, self.uav_y, self.uav_heading)
        print("仿真已重置。目标位置:", (int(self.target_x), int(self.target_y)))

    def close(self) -> None:
        try:
            if hasattr(self, "csv_file") and self.csv_file:
                self.csv_file.close()
        finally:
            pygame.quit()

    def handle_events(self) -> None:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False
                elif event.key == pygame.K_SPACE:
                    self.paused = not self.paused
                elif event.key == pygame.K_r:
                    self.reset()

    def _build_scene_surface(self) -> pygame.Surface:
        scene = self.map_surface.copy()
        target_rect = self.target_image.get_rect(center=(int(self.target_x), int(self.target_y)))
        scene.blit(self.target_image, target_rect)

        if self.target_found:
            pygame.draw.circle(
                scene,
                cfg.COLOR_GREEN,
                (int(self.target_x), int(self.target_y)),
                cfg.TARGET_SIZE // 2 + 6,
                3,
            )
        return scene

    def _update_uav_position(self, next_x: float, next_y: float) -> None:
        dx = next_x - self.uav_x
        dy = next_y - self.uav_y
        if dx != 0.0 or dy != 0.0:
            self.uav_heading = math.degrees(math.atan2(dy, dx)) % 360.0
        self.distance_traveled += math.sqrt(dx ** 2 + dy ** 2)
        self.uav_x = next_x
        self.uav_y = next_y
        self.trajectory.append((self.uav_x, self.uav_y))
        if len(self.trajectory) > cfg.MAX_TRAJECTORY_POINTS:
            self.trajectory.pop(0)

    def _check_target_detected(self) -> bool:
        distance = math.sqrt((self.target_x - self.uav_x) ** 2 + (self.target_y - self.uav_y) ** 2)
        return distance <= cfg.UAV_DETECTION_RADIUS

    def _write_log(self) -> None:
        mode = self.last_debug.get("模式", "未知")
        score_now = float(self.last_debug.get("匹配分数", 0.0))
        score_smooth = float(self.last_debug.get("平滑分数", 0.0))
        locked = self.last_debug.get("目标是否锁定", "否")

        self.csv_writer.writerow(
            [
                self.frames,
                mode,
                f"{score_now:.4f}",
                f"{score_smooth:.4f}",
                int(self.uav_x),
                int(self.uav_y),
                locked,
                "是" if self.target_found else "否",
            ]
        )

        if self.frames % cfg.PRINT_LOG_INTERVAL == 0:
            print(
                f"帧号={self.frames:05d}, 模式={mode}, 匹配分数={score_now:.4f}, "
                f"平滑分数={score_smooth:.4f}, 位置=({int(self.uav_x)}, {int(self.uav_y)}), "
                f"目标是否锁定={locked}"
            )

    def update(self) -> None:
        if self.paused or self.search_complete:
            return

        scene = self._build_scene_surface()
        next_x, next_y = self.algorithm.get_next_position(
            self.uav_x,
            self.uav_y,
            self.uav_heading,
            scene_surface=scene,
        )

        self._update_uav_position(next_x, next_y)
        self.last_debug = self.algorithm.get_debug_info()

        if self._check_target_detected():
            self.target_found = True
            self.search_complete = True
            print(
                f"目标已发现！帧数={self.frames}, 用时={self.frames / cfg.FPS:.2f}秒, "
                f"飞行距离={self.distance_traveled:.2f}像素"
            )

        self._write_log()
        self.frames += 1

        if self.frames >= cfg.MAX_FRAMES:
            self.search_complete = True
            print(f"达到最大帧数限制 {cfg.MAX_FRAMES}，仿真停止。")

    def _draw_target(self, surface: pygame.Surface) -> None:
        target_rect = self.target_image.get_rect(center=(int(self.target_x), int(self.target_y)))
        surface.blit(self.target_image, target_rect)
        if self.target_found:
            pygame.draw.circle(
                surface,
                cfg.COLOR_GREEN,
                (int(self.target_x), int(self.target_y)),
                cfg.TARGET_SIZE // 2 + 6,
                3,
            )

    def _draw_uav(self, surface: pygame.Surface) -> None:
        if cfg.SHOW_DETECTION_RANGE:
            overlay = pygame.Surface((cfg.WINDOW_WIDTH, cfg.WINDOW_HEIGHT), pygame.SRCALPHA)
            pygame.draw.circle(
                overlay,
                cfg.COLOR_DETECTION_RANGE,
                (int(self.uav_x), int(self.uav_y)),
                cfg.UAV_DETECTION_RADIUS,
            )
            surface.blit(overlay, (0, 0))
            pygame.draw.circle(
                surface,
                cfg.COLOR_GREEN,
                (int(self.uav_x), int(self.uav_y)),
                cfg.UAV_DETECTION_RADIUS,
                1,
            )

        if cfg.SHOW_TRAJECTORY and len(self.trajectory) > 1:
            pygame.draw.lines(
                surface,
                cfg.TRAJECTORY_COLOR,
                False,
                [(int(x), int(y)) for x, y in self.trajectory],
                cfg.TRAJECTORY_WIDTH,
            )

        rotated = pygame.transform.rotate(self.uav_image, -self.uav_heading)
        rect = rotated.get_rect(center=(self.uav_x, self.uav_y))
        surface.blit(rotated, rect)

    def _draw_algorithm_overlay(self, surface: pygame.Surface) -> None:
        roi_rect = self.last_debug.get("ROI矩形")
        peak = self.last_debug.get("峰值坐标")
        mode = self.last_debug.get("模式", "未知")

        if roi_rect:
            x1, y1, x2, y2 = roi_rect
            pygame.draw.rect(surface, cfg.COLOR_YELLOW, pygame.Rect(x1, y1, x2 - x1, y2 - y1), 2)

        if peak is not None:
            pygame.draw.circle(surface, cfg.COLOR_RED, (int(peak[0]), int(peak[1])), 6, 2)

        mode_label = self.font.render(f"当前模式: {mode}", True, cfg.COLOR_CYAN)
        surface.blit(mode_label, (cfg.INFO_PANEL_X, cfg.WINDOW_HEIGHT - 40))

    def _draw_info_panel(self, surface: pygame.Surface) -> None:
        mode = self.last_debug.get("模式", "巡航模式")
        score_now = float(self.last_debug.get("匹配分数", 0.0))
        score_smooth = float(self.last_debug.get("平滑分数", 0.0))
        locked = self.last_debug.get("目标是否锁定", "否")
        status = "已发现目标" if self.target_found else ("暂停中" if self.paused else "搜索中")

        lines = [
            f"算法: {cfg.ALGORITHM_NAME}",
            f"时间帧: {self.frames}",
            f"时间(秒): {self.frames / cfg.FPS:.2f}",
            f"飞行距离: {self.distance_traveled:.2f} 像素",
            f"当前位置: ({int(self.uav_x)}, {int(self.uav_y)})",
            f"模式: {mode}",
            f"匹配分数: {score_now:.4f}",
            f"平滑分数: {score_smooth:.4f}",
            f"目标是否锁定: {locked}",
            f"状态: {status}",
            "控制: 空格暂停/继续, R重置, ESC退出",
        ]

        line_height = cfg.INFO_FONT_SIZE + 4
        panel_width = 520
        panel_height = len(lines) * line_height + 16
        panel = pygame.Surface((panel_width, panel_height), pygame.SRCALPHA)
        panel.fill(cfg.INFO_BG_COLOR)
        surface.blit(panel, (cfg.INFO_PANEL_X, cfg.INFO_PANEL_Y))

        y = cfg.INFO_PANEL_Y + 8
        for line in lines:
            text = self.font.render(line, True, cfg.INFO_TEXT_COLOR)
            surface.blit(text, (cfg.INFO_PANEL_X + 8, y))
            y += line_height

    def _save_annotated_image(self) -> None:
        if self.frames % cfg.SAVE_IMAGE_INTERVAL != 0:
            return
        mode = self.last_debug.get("模式", "未知模式")
        score = float(self.last_debug.get("匹配分数", 0.0))
        filename = f"第{self.frames:05d}帧_{mode}_匹配分数{score:.2f}.png"
        file_path = os.path.join(self.image_dir, filename)
        pygame.image.save(self.screen, file_path)

    def draw(self) -> None:
        self.screen.blit(self.map_surface, (0, 0))
        self._draw_target(self.screen)
        self._draw_uav(self.screen)
        self._draw_algorithm_overlay(self.screen)
        self._draw_info_panel(self.screen)
        pygame.display.flip()

    def run(self) -> None:
        print("启动独立基线仿真:", cfg.ALGORITHM_NAME)
        print("目标模板路径:", cfg.TARGET_TEMPLATE_PATH)
        print("按 ESC 退出，按 SPACE 暂停/继续，按 R 重置")
        try:
            while self.running:
                self.handle_events()
                self.update()
                self.draw()
                self._save_annotated_image()
                self.clock.tick(cfg.FPS)
        finally:
            self.close()
            print("仿真已结束。")

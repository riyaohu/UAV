"""
Simulator - main simulation engine
"""
# TODO (evaluation): UAV start position policy (fixed vs random_free)
# Will be unified later for fair comparison across algorithms

import pygame
from . import config
from .map_manager import MapManager
from .uav import UAV
from .target import Target
from .algorithms import RandomSearch, CoverageSearch, FrontierSearch, InformationGainSearch
from .environment.grid_map import GridMap
from .observation.observation_model import ObservationModel
from .belief.grid_belief import GridBelief
import os
from .baseline_template_matching_cruise import config_template_matching as tm_cfg
from .baseline_template_matching_cruise.template_matching_cruise import TemplateMatchingCruise


class Simulator:
    """Main simulator class that manages the simulation"""

    def __init__(self, render=True, mode="experiment", use_grid_map=None, algorithm_name=None, start_pos=None):
        """Initialize the simulator"""
        # use_grid_map 参数优先；如果没传（None），才使用 config 默认值
        if use_grid_map is None:
            self.use_grid_map = getattr(config, "USE_GRID_MAP", False)
        else:
            self.use_grid_map = use_grid_map

        # Initialize pygame
        # Initialize pygame
        self.render = render
        self.mode = mode
        self.stop_reason = None

        # TemplateMatching 即使在 render=False 时，也需要 pygame 加载/处理图像
        if algorithm_name == "template_matching" and not self.render:
            os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
            pygame.init()
            pygame.display.set_mode((1, 1))

        if self.render:
            pygame.init()

            # Create window
            w = config.WINDOW_WIDTH
            h = config.WINDOW_HEIGHT
            if self.use_grid_map:
                w = config.MAP_WIDTH
                h = config.MAP_HEIGHT
            self.screen = pygame.display.set_mode((w, h))

            pygame.display.set_caption(config.WINDOW_TITLE)
            self.clock = pygame.time.Clock()

            # Initialize font
            self.font = pygame.font.Font(None, config.INFO_FONT_SIZE)

        # ========== Map source selection (GridMap vs Image Map) ==========
        self.grid_map = None
        self.map_manager = None

        if self.use_grid_map:
            # 1) 使用参数化栅格地图
            self.map_width = config.MAP_WIDTH
            self.map_height = config.MAP_HEIGHT

            self.grid_map = GridMap(
                width_px=self.map_width,
                height_px=self.map_height,
                cell_size=config.CELL_SIZE,
                obstacle_density=config.OBSTACLE_DENSITY,
                border_blocked=getattr(config, "BORDER_BLOCKED", False),
            )
        else:
            # 2) 使用图片地图（旧方式）
            self.map_manager = MapManager()
            self.map_width, self.map_height = self.map_manager.get_size()

        # --- B1: 保留图片背景用于渲染（只在 render=True 时加载）---
        if self.render and self.map_manager is None:
            self.map_manager = MapManager()

        # ========== Create UAV ==========
        # NOTE: UAV start position is randomized but controlled by experiment seed
        #       This ensures fair comparison across algorithms
        # UAV 初始点：如果是栅格地图，尽量落在free cell上（避免一开始就在障碍里）
        # ========== Create UAV ==========
        # algorithm_name 先规范化（这里不会改变你后面选择算法的逻辑）
        if algorithm_name is None:
            algorithm_name = getattr(config, "DEFAULT_ALGORITHM", "random")

        self.algorithm_name = algorithm_name
        # ===== 公平实验：所有算法使用相同终止条件 =====
        self.max_frames = config.MAX_FRAMES
        self.max_distance = config.MAX_DISTANCE
        # UAV 初始点：栅格地图下，按算法选择起点策略
        if self.grid_map is not None:
            if start_pos is not None:
                start_x, start_y = start_pos
            else:
                start_x, start_y = self.grid_map.random_free_position()

            self.uav = UAV(start_x, start_y, height=config.UAV_HEIGHT)
        else:
            self.uav = UAV(config.UAV_START_X, config.UAV_START_Y)

        self.start_x = self.uav.start_x
        self.start_y = self.uav.start_y

        # ========== Observation model (Step 7) ==========
        self.observer = ObservationModel(
            p_false_negative=getattr(config, "P_FALSE_NEGATIVE", 0.0),
            p_false_positive=getattr(config, "P_FALSE_POSITIVE", 0.0),
            distance_noise_std=getattr(config, "DISTANCE_NOISE_STD", 0.0),
        )
        self.last_false_positive = None  # 记录最近一次误检点（可选）

        # ========== Create targets ==========
        self.targets = []
        for i in range(config.NUM_TARGETS):
            x, y = config.TARGET_POSITIONS[i]
            self.targets.append(Target(x, y))

        # ========== Initialize algorithm ==========
        if algorithm_name is None:
            algorithm_name = getattr(config, "DEFAULT_ALGORITHM", "random")

        if algorithm_name == "coverage":
            step = getattr(config, "COVERAGE_STEP", 20)

            # 新增：扫描线间距，默认按探测半径来
            lane_spacing = getattr(config, "COVERAGE_LANE_SPACING", int(self.uav.detection_radius * 2 * 0.9))

            self.algorithm = CoverageSearch(self.map_width, self.map_height, step=step, lane_spacing=lane_spacing)


        elif algorithm_name == "frontier":

            self.algorithm = FrontierSearch(
                map_width=self.map_width,
                map_height=self.map_height,
                speed=config.UAV_SPEED,
                gain_radius_cells=4,
                lambda_cost=0.002
            )


        elif algorithm_name == "information_gain":

            self.algorithm = InformationGainSearch(

                map_width=self.map_width,

                map_height=self.map_height,

                speed=config.UAV_SPEED,

                num_directions=16,

                lambda_turn=0.8,

                switch_margin=0.08,

                hold_steps_max=10,

                lambda_boundary=2.0

            )

        elif algorithm_name == "template_matching":

            if self.use_grid_map:
                raise ValueError("template_matching 只能在图片地图模式下运行，请设置 use_grid_map=False")

            self.algorithm = TemplateMatchingCruise(
                self.map_width,
                self.map_height,
                tm_cfg
            )

            self.algorithm.reset(
                self.uav.x,
                self.uav.y,
                self.uav.angle
            )


        else:
            self.algorithm = RandomSearch(self.map_width, self.map_height)

        # 由算法声明决定是否启用 belief（RandomSearch 默认为 False）
        self.enable_belief = getattr(self.algorithm, "uses_belief", False)
        # 只有在使用栅格地图 且 算法需要 belief 时，才创建 belief（避免 runner 变慢）
        self.belief = None
        if self.grid_map is not None and self.enable_belief:
            self.belief = GridBelief(self.grid_map)

        # Simulation state
        self.running = True
        self.paused = False
        self.target_found = False
        self.search_complete = False

        # Statistics
        self.frames = 0
        # ===== 新增：逐帧日志 =====
        self.frame_logs = []
        # Step 7 stats
        self.num_false_negatives = 0
        self.num_false_positives = 0

        # Step 7 visualization (false positive TTL)
        self.fp_pos = None
        self.fp_ttl = 0

    def handle_events(self):
        """Handle user input events"""
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

    def build_template_scene_surface(self):
        """
        为 TemplateMatchingCruise 构造图像场景：
        地图背景 + 目标图像。
        不画 UAV、不画轨迹、不画信息面板。
        这样最接近老师原始 comparison_runner 的输入。
        """
        if self.map_manager is None:
            raise ValueError("template_matching 需要图片地图 map_manager")

        surface = self.map_manager.image.copy()

        for t in self.targets:
            if hasattr(t, "image") and t.image is not None:
                rect = t.image.get_rect(center=(int(t.x), int(t.y)))
                surface.blit(t.image, rect)
            else:
                pygame.draw.circle(surface, config.COLOR_RED, (int(t.x), int(t.y)), 8)

        return surface

    def update(self):
        """Update simulation state"""
        if self.paused or self.search_complete:
            return

        # Get next position from algorithm
        current_x, current_y = self.uav.get_position()

        if isinstance(self.algorithm, TemplateMatchingCruise):
            scene_surface = self.build_template_scene_surface()

            next_x, next_y = self.algorithm.get_next_position(
                current_x,
                current_y,
                self.uav.angle,
                scene_surface=scene_surface
            )
        else:
            next_x, next_y = self.algorithm.get_next_position(
                current_x, current_y, self.uav.angle,
                grid_map=self.grid_map,
                belief=self.belief,
                detection_radius=self.uav.detection_radius
            )

        # --- Step: enforce max step length (avoid teleport) ---
        dx = next_x - current_x
        dy = next_y - current_y
        step = (dx * dx + dy * dy) ** 0.5
        max_step = self.uav.speed  # config.UAV_SPEED

        if step > max_step and step > 1e-9:
            scale = max_step / step
            next_x = current_x + dx * scale
            next_y = current_y + dy * scale

        # Update UAV position with obstacle check
        if self.grid_map is not None and self.grid_map.is_blocked_px(next_x, next_y):

            # 撞到障碍：最简单处理——原地不动
            # （为了避免一直卡住，可以让随机搜索重置一下方向）
            # 对随机类算法：reset 可以帮助脱困
            if isinstance(self.algorithm, RandomSearch):
                self.algorithm.reset()

            # 所有算法：撞障碍就原地不动（由算法自己决定下一步怎么绕）
            next_x, next_y = self.uav.get_position()

        self.uav.set_position(next_x, next_y)
        # --- mark explored area for frontier search ---
        if self.enable_belief and self.belief is not None:
            self.belief.mark_explored_in_radius(self.uav.x, self.uav.y, self.uav.detection_radius)

        # ===== Step 7: Observation-based detection (noisy / FN / FP) =====

        # 逐个检测目标：使用 observer，而不是完美检测
        # ===== Step 7: Observation-based detection + counters =====

        detected_any = False

        # 逐个检测目标：用观测模型（带漏检/噪声）
        for t in self.targets:
            if t.found:
                continue

            obs = self.observer.observe_target(
                uav_x=self.uav.x, uav_y=self.uav.y,uav_height=self.uav.height,
                target_x=t.x, target_y=t.y,
                detection_radius=self.uav.detection_radius
            )

            if obs["detected"]:
                t.found = True
                detected_any = True
                if self.enable_belief and self.belief is not None:
                    self.belief.mark_found(t.x, t.y)

                print(
                    f"Detected target at ({t.x}, {t.y})! "
                    f"true_d={obs['true_distance']:.1f}, noisy_d={obs['noisy_distance']:.1f}. "
                    f"Now: {self.get_found_count()}/{len(self.targets)}"
                )
            else:
                # “漏检”计数：真实距离在半径内，但没检测到
                if obs["true_distance"] <= self.uav.detection_radius:
                    self.num_false_negatives += 1

        if self.enable_belief and (not detected_any) and (self.belief is not None):
            decay = getattr(config, "BELIEF_NEGATIVE_DECAY", 0.5)
            self.belief.update_negative(self.uav.x, self.uav.y, self.uav.detection_radius, decay=decay)

        # ===== Step 7: False positive + TTL (for visualization) =====
        fp = self.observer.maybe_false_positive(self.map_width, self.map_height)
        if fp is not None:
            self.num_false_positives += 1
            self.fp_pos = fp["pos"]
            self.fp_ttl = getattr(config, "FALSE_POSITIVE_TTL", 30)
        else:
            # 没产生新的误检时，让旧红点慢慢“过期”
            if self.fp_ttl > 0:
                self.fp_ttl -= 1
                if self.fp_ttl == 0:
                    self.fp_pos = None

        # 是否全部找到
        if config.STOP_WHEN_ALL_FOUND:
            if self.get_found_count() == len(self.targets):
                self.target_found = True  # 你可以继续沿用这个变量名：含义改成“全部找到”
                self.search_complete = True
                print(
                    f"All targets found! Time: {self.frames} frames, Distance: {self.uav.distance_traveled:.2f} pixels")

        # ===== 新增：逐帧日志记录 =====
        self.frame_logs.append({
            "frame": self.frames,
            "x": self.uav.x,
            "y": self.uav.y,
            "distance": self.uav.distance_traveled,
            "found_count": self.get_found_count(),
            "target_found": int(self.target_found),
        })

        self.frames += 1

        stop, reason = self.check_termination()
        if stop:
            self.stop_reason = reason
            if self.mode == "experiment":
                self.running = False
            elif self.mode == "demo":
                self.paused = True  # 停下来但不退出

    def get_found_count(self):
        return sum(1 for t in self.targets if t.found)

    def get_frame_logs(self):
        return self.frame_logs

    def draw_template_matching_overlay(self):
        if not isinstance(self.algorithm, TemplateMatchingCruise):
            return

        info = self.algorithm.get_debug_info()

        roi_rect = info.get("ROI矩形")
        peak = info.get("峰值坐标")

        # ===== 画 ROI（黄色框）=====
        if roi_rect is not None:
            x1, y1, x2, y2 = roi_rect
            pygame.draw.rect(
                self.screen,
                (255, 255, 0),  # 黄色
                pygame.Rect(x1, y1, x2 - x1, y2 - y1),
                2
            )

        # ===== 画匹配点（红点）=====
        if peak is not None:
            pygame.draw.circle(
                self.screen,
                (255, 0, 0),
                (int(peak[0]), int(peak[1])),
                6
            )


    def draw(self):
        """Draw all elements on screen"""
        # Draw background: image map if available, else white
        if self.map_manager is not None:
            self.map_manager.draw(self.screen)
        else:
            self.screen.fill(config.COLOR_WHITE)

        if self.grid_map is not None:
            # 新模式：绘制障碍格
            for r in range(self.grid_map.rows):
                for c in range(self.grid_map.cols):
                    if self.grid_map.grid[r][c] == 1:
                        rect = pygame.Rect(
                            c * self.grid_map.cell_size,
                            r * self.grid_map.cell_size,
                            self.grid_map.cell_size,
                            self.grid_map.cell_size
                        )
                        pygame.draw.rect(self.screen, config.COLOR_YELLOW, rect)
                        pygame.draw.rect(self.screen, config.COLOR_BLACK, rect, width=1)

        if self.enable_belief and getattr(config, "SHOW_BELIEF",
                                          False) and self.belief is not None and self.grid_map is not None:

            alpha = getattr(config, "BELIEF_ALPHA", 180)  # 你可以先用 220 更明显

            # 1) 找最大概率用于归一化显示
            max_p = 0.0
            for r in range(self.grid_map.rows):
                for c in range(self.grid_map.cols):
                    if self.grid_map.grid[r][c] == 0:
                        p = self.belief.belief[r][c]
                        if p > max_p:
                            max_p = p

            if max_p > 1e-12:
                for r in range(self.grid_map.rows):
                    for c in range(self.grid_map.cols):
                        if self.grid_map.grid[r][c] == 1:
                            continue

                        # 2) 归一化到 0~1
                        p = self.belief.belief[r][c] / max_p

                        # 3) 用 gamma 提升对比度：让“高概率”更红
                        gamma = getattr(config, "BELIEF_GAMMA", 0.35)  # 越小越显眼
                        intensity = p ** gamma  # 0~1

                        # 4) 颜色映射：红色强=概率高；透明度固定
                        red = int(255 * intensity)
                        green = int(40 * (1 - intensity))  # 少量绿，让颜色更“热”
                        blue = int(40 * (1 - intensity))

                        rect = pygame.Rect(
                            c * self.grid_map.cell_size,
                            r * self.grid_map.cell_size,
                            self.grid_map.cell_size,
                            self.grid_map.cell_size
                        )
                        s = pygame.Surface((rect.width, rect.height), pygame.SRCALPHA)
                        s.fill((255, 0, 0, int(alpha * intensity)))  # 关键：透明度也随概率变化
                        self.screen.blit(s, rect.topleft)

        # Step 7: draw false positive point (optional)
        if getattr(config, "SHOW_FALSE_POSITIVES", True):
            if self.fp_pos is not None and self.fp_ttl > 0:
                x, y = self.fp_pos
                pygame.draw.circle(self.screen, config.COLOR_RED, (int(x), int(y)), 6)

        # Draw target (only if not found, or with highlight if found)
        for t in self.targets:
            t.draw(self.screen)

        # Draw UAV with detection range and trajectory
        self.uav.draw(
            self.screen,
            show_detection_range=config.SHOW_DETECTION_RANGE,
            show_trajectory=config.SHOW_TRAJECTORY
        )
        self.draw_template_matching_overlay()
        # Draw info panel
        self.draw_info_panel()

        # Update display
        pygame.display.flip()

    def draw_info_panel(self):
        """Draw information panel with statistics"""
        found_count = self.get_found_count()
        info_lines = [
            f"Algorithm: {self.algorithm.get_name()}",
            f"Time: {self.frames} frames ({self.frames / config.FPS:.1f}s)",
            f"Distance: {self.uav.distance_traveled:.1f} px",
            f"Position: ({int(self.uav.x)}, {int(self.uav.y)})",
            f"Found: {found_count}/{len(self.targets)}",
            f"False Negatives: {self.num_false_negatives}",
            f"Status: {'ALL FOUND!' if self.target_found else 'Searching...'}"
        ]

        if self.paused:
            info_lines.append("PAUSED")

        # Calculate panel dimensions
        line_height = config.INFO_FONT_SIZE + 5
        panel_height = len(info_lines) * line_height + 20
        panel_width = 350

        # Draw semi-transparent background
        info_surface = pygame.Surface((panel_width, panel_height), pygame.SRCALPHA)
        info_surface.fill(config.INFO_BG_COLOR)
        self.screen.blit(info_surface, (config.INFO_PANEL_X, config.INFO_PANEL_Y))

        # Draw text lines
        y_offset = config.INFO_PANEL_Y + 10
        for line in info_lines:
            text_surface = self.font.render(line, True, config.INFO_TEXT_COLOR)
            self.screen.blit(text_surface, (config.INFO_PANEL_X + 10, y_offset))
            y_offset += line_height

        # Draw controls help at bottom
        help_lines = [
            "Controls:",
            "SPACE - Pause/Resume",
            "R - Reset",
            "ESC - Quit"
        ]

        # help_y = config.WINDOW_HEIGHT - len(help_lines) * line_height - 20
        screen_h = self.screen.get_height()
        help_y = screen_h - len(help_lines) * line_height - 20

        help_surface = pygame.Surface((panel_width, len(help_lines) * line_height + 20), pygame.SRCALPHA)
        help_surface.fill(config.INFO_BG_COLOR)
        self.screen.blit(help_surface, (config.INFO_PANEL_X, help_y))

        y_offset = help_y + 10
        for line in help_lines:
            text_surface = self.font.render(line, True, config.INFO_TEXT_COLOR)
            self.screen.blit(text_surface, (config.INFO_PANEL_X + 10, y_offset))
            y_offset += line_height

    def reset(self):
        """Reset simulation to initial state"""
        self.uav.reset()
        for t in self.targets:
            t.reset()
        if isinstance(self.algorithm, TemplateMatchingCruise):
            self.algorithm.reset(self.uav.x, self.uav.y, self.uav.angle)
        else:
            self.algorithm.reset()
        self.paused = False
        self.running = True
        self.target_found = False
        self.search_complete = False
        self.stop_reason = None
        self.frames = 0
        print("Simulation reset")
        # Step 7: reset sensor stats & fp visualization
        self.num_false_negatives = 0
        self.num_false_positives = 0
        self.fp_pos = None
        self.fp_ttl = 0
        if self.belief is not None:
            self.belief.reset()

    def run(self):
        """Main simulation loop"""
        print(f"Starting UAV Search Simulator")
        print(f"Algorithm: {self.algorithm.get_name()}")
        print(f"Targets: {[ (t.x, t.y) for t in self.targets ]}")
        print(f"Detection radius: {self.uav.detection_radius} pixels")
        print("\nControls:")
        print("  SPACE - Pause/Resume")
        print("  R - Reset")
        print("  ESC - Quit")


        while self.running:
            # Handle events
            if self.render:
                self.handle_events()

            # Update simulation
            self.update()

            # Draw everything
            if self.render:
                self.draw()
            # Control frame rate
                self.clock.tick(config.FPS)

        # Cleanup
        if self.render:
            pygame.quit()
        print(f"Stop reason: {self.stop_reason}")
        print(f"Frames: {self.frames}")
        print("Simulator closed")

    def set_algorithm(self, algorithm):
        """
        Change the search algorithm

        Args:
            algorithm: New algorithm instance
        """
        self.algorithm = algorithm
        self.reset()

    # 停止条件
    def check_termination(self):
        if self.target_found:
            return True, "success"

        if self.frames >= self.max_frames:
            return True, "timeout"

        if self.uav.distance_traveled >= self.max_distance:
            return True, "distance_limit"

        return False, None


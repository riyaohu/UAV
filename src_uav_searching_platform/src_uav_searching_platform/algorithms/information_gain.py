# algorithms/information_gain.py
import math
import random
from .base_algorithm import BaseAlgorithm


class InformationGainSearch(BaseAlgorithm):
    """
    Information Gain (one-step lookahead) baseline.

    关键改进（为了解决你遇到的：贴边半圆 + 障碍/边界卡死）：
    1) gain 的评估半径使用真实 detection_radius（像素）换算到 cell 半径，而不是写死 gain_radius_cells
    2) gain 乘以“有效覆盖比例 cover_ratio”（越贴边/越靠角落，覆盖圆落在地图外越多 → gain 自动打折）
    3) try_step_with_avoid 同时尝试缩短步长（speed, 0.7*speed, 0.4*speed），避免在窄空间一步走不出去
    4) 抗抖动：方向惯性（hold_steps）+ 转向惩罚（lambda_turn）+ 切换阈值（switch_margin）
    5) 连续卡住检测：连续 N 帧几乎没移动 → 强制 center_escape
    """
    uses_belief = True

    def __init__(
        self,

        map_width,
        map_height,
        speed=3.0,
        num_directions=16,
        lambda_turn=0.8,
        switch_margin=0.08,
        hold_steps_max=10,
        lambda_boundary=0.0,
    ):
        print("[IG] loaded from:", __file__)

        super().__init__(map_width, map_height)
        self.name = "Information Gain"
        self.speed = float(speed)
        self.num_directions = int(num_directions)

        # --- anti-oscillation / inertia ---
        self.prev_angle = None
        self.hold_steps = 0
        self.hold_steps_max = int(hold_steps_max)
        self.switch_margin = float(switch_margin)  # 新方向必须明显更好才切换
        self.lambda_turn = float(lambda_turn)      # 转向惩罚（越大越不容易左右抖）

        # --- boundary ---
        # 这里保留一个基础边界惩罚（可选）。真正让贴边无利可图的是 estimate_gain() 里的 cover_ratio 折扣。
        self.lambda_boundary = float(lambda_boundary)

        # --- stuck detection ---
        self.stuck_count = 0
        self.last_pos = None
        self.stuck_eps = 0.5       # 小于0.5px 视为没动
        self.stuck_trigger = 8    # 连续15帧没动就触发逃逸

        # --- escape threshold ---
        self.min_gain_escape = 1e-9

    def reset(self):
        self.prev_angle = None
        self.hold_steps = 0
        self.stuck_count = 0
        self.last_pos = None

    # ---------------- core ----------------

    def get_next_position(self, current_x, current_y, current_angle, **kwargs):
        belief = kwargs.get("belief", None)
        grid_map = kwargs.get("grid_map", None)
        det_r = kwargs.get("detection_radius", None)
        cell = float(getattr(grid_map, "cell_size", 20.0))
        # 评估用更远的lookahead距离：至少2个cell，或0.6个探测半径
        lookahead = max(2.0 * cell, 0.6 * float(det_r))

        # 兜底
        if belief is None or grid_map is None or det_r is None:
            return self.random_step(current_x, current_y)

        # 1) stuck detection：连续多帧几乎没动 → 强制往中心逃逸
        if self.last_pos is None:
            self.last_pos = (current_x, current_y)
        moved = math.hypot(current_x - self.last_pos[0], current_y - self.last_pos[1])
        if moved < self.stuck_eps:
            self.stuck_count += 1
        else:
            self.stuck_count = 0
        self.last_pos = (current_x, current_y)

        if self.stuck_count >= self.stuck_trigger:
            self.stuck_count = 0
            nx, ny, ang = self.center_escape(current_x, current_y, grid_map)
            if nx is not None:
                self.prev_angle = ang
                self.hold_steps = self.hold_steps_max
                return nx, ny
            # 如果中心也走不通，就随机（仍避障）
            ang = random.uniform(0, 2 * math.pi)
            nx, ny = self.try_step_with_avoid(current_x, current_y, ang, grid_map)
            if nx is not None:
                self.prev_angle = ang
                self.hold_steps = self.hold_steps_max
                return nx, ny
            return current_x, current_y

        # 2) 方向惯性：如果还在“坚持方向”，优先沿用
        if self.prev_angle is not None and self.hold_steps > 0:
            nx, ny = self.try_step_with_avoid(current_x, current_y, self.prev_angle, grid_map)
            if nx is not None:
                self.hold_steps -= 1
                return nx, ny
            else:
                # 走不通就取消坚持
                self.hold_steps = 0

        # 3) 计算 keep（沿用 prev_angle）的 gain，用于 hysteresis
        keep_gain = None
        keep_next = None
        if self.prev_angle is not None:
            kx, ky = self.try_step_with_avoid(current_x, current_y, self.prev_angle, grid_map)
            if kx is not None:
                ex = current_x + lookahead * math.cos(self.prev_angle)
                ey = current_y + lookahead * math.sin(self.prev_angle)
                ex = min(max(ex, 0.0), self.map_width)
                ey = min(max(ey, 0.0), self.map_height)
                keep_gain = self.estimate_gain(belief, ex, ey, det_r)
                keep_next = (kx, ky)

        # 4) 枚举方向，选择 best
        best_score = -1e18
        best_gain = -1e18
        best_next = None
        best_angle = None

        for i in range(self.num_directions):
            ang = 2 * math.pi * i / self.num_directions
            # 实际移动点：仍然只走一步（让Simulator限制生效）
            nx, ny = self.try_step_with_avoid(current_x, current_y, ang, grid_map)
            if nx is None:
                continue

            # 评估点：沿该方向看得更远（决定方向）
            ex = current_x + lookahead * math.cos(ang)
            ey = current_y + lookahead * math.sin(ang)

            # clamp到地图内（避免越界导致评估无意义）
            ex = min(max(ex, 0.0), self.map_width)
            ey = min(max(ey, 0.0), self.map_height)

            gain = self.estimate_gain(belief, ex, ey, det_r)
            bpen = self.boundary_penalty(ex, ey, det_r)

            # -------- turn cost（转向惩罚）--------
            if self.prev_angle is not None:
                # 角度差归一化到 [0, 1]
                turn_cost = abs(self._angle_diff(ang, self.prev_angle)) / math.pi
            else:
                turn_cost = 0.0

            score = gain - self.lambda_turn * turn_cost - self.lambda_boundary * bpen

            if score > best_score:
                best_score = score
                best_gain = gain
                best_next = (nx, ny)
                best_angle = ang

        # 5) 如果 best_gain 太小：说明局部基本没有新信息了 → 逃逸
        if best_next is None or best_gain < self.min_gain_escape:
            nx, ny, ang = self.center_escape(current_x, current_y, grid_map)
            if nx is not None:
                self.prev_angle = ang
                self.hold_steps = self.hold_steps_max
                return nx, ny

            ang = random.uniform(0, 2 * math.pi)
            nx, ny = self.try_step_with_avoid(current_x, current_y, ang, grid_map)
            if nx is not None:
                self.prev_angle = ang
                self.hold_steps = self.hold_steps_max
                return nx, ny
            return current_x, current_y

        # 6) hysteresis：新方向必须明显比 keep_gain 好，才切换（否则沿用）
        if keep_gain is not None:
            if best_gain <= keep_gain * (1.0 + self.switch_margin):
                self.prev_angle = self.prev_angle
                self.hold_steps = self.hold_steps_max
                return keep_next

        # 7) 选择 best
        self.prev_angle = best_angle
        self.hold_steps = self.hold_steps_max
        return best_next

    # ---------------- gain model ----------------

    def estimate_gain(self, belief, px, py, det_r):
        """
        InfoGain 近似：
          - 以 UAV 探测半径 det_r（像素）为基准换算出 cell 半径 R
          - 在 (px,py) 的覆盖区域附近统计“未探索 cell”的 belief 概率质量
          - 乘以 cover_ratio 折扣：越贴边/角落，覆盖圆落在地图外越多 → gain 越小
        """
        r0, c0 = belief.grid_map.px_to_cell(px, py)
        cell = float(getattr(belief.grid_map, "cell_size", 20.0))
        R = max(1, int(float(det_r) / max(1e-9, cell)))  # 用真实探测半径换算

        s = 0.0
        scanned = 0

        r_min = max(0, r0 - R)
        r_max = min(belief.rows, r0 + R + 1)
        c_min = max(0, c0 - R)
        c_max = min(belief.cols, c0 + R + 1)

        for rr in range(r_min, r_max):
            for cc in range(c_min, c_max):
                scanned += 1
                if belief.grid_map.is_blocked_cell(rr, cc):
                    continue
                if belief.is_unexplored(rr, cc):
                    s += belief.belief[rr][cc]

        # 覆盖折扣：理想覆盖面积 ~ pi*R^2；实际 scanned 因边界裁剪会变小
        # scanned 是实际能“看见”的窗口 cell 数（靠边会变小）
        max_scanned = (2 * R + 1) * (2 * R + 1)  # 地图内部的最大窗口大小
        cover_ratio = scanned / max(1e-9, max_scanned)  # 0~1，越贴边越小

        return s * cover_ratio

    # ---------------- movement helpers ----------------

    def try_step_with_avoid(self, x, y, angle, grid_map):
        """
        在期望方向 angle 上走一步；如果无效则扇形试探。
        同时尝试缩短步长，避免在边界/障碍旁“一步跨不过去”导致无路可走。
        """
        step_scales = [1.0, 0.7, 0.4]

        def attempt(ang):
            for sc in step_scales:
                nx = x + (self.speed * sc) * math.cos(ang)
                ny = y + (self.speed * sc) * math.sin(ang)
                if self.is_valid(nx, ny, grid_map):
                    return nx, ny
            return None, None

        nx, ny = attempt(angle)
        if nx is not None:
            return nx, ny

        delta = math.pi / 12  # 15度
        for k in range(1, 13):
            for sgn in (+1, -1):
                ang = angle + sgn * k * delta
                nx, ny = attempt(ang)
                if nx is not None:
                    return nx, ny

        return None, None

    def is_valid(self, x, y, grid_map):
        if x < 0 or x > self.map_width or y < 0 or y > self.map_height:
            return False
        if grid_map.is_blocked_px(x, y):
            return False
        return True

    def boundary_penalty(self, x, y, det_r):
        """
        可选的边界惩罚（0~1）。核心贴边抑制来自 estimate_gain 的 cover_ratio 折扣。
        这里额外给一个“越靠边越扣分”的项，det_r 用作惩罚范围尺度。
        """
        d = min(x, y, self.map_width - x, self.map_height - y)
        margin = float(det_r) if det_r is not None else 0.0
        if margin <= 1e-6:
            return 0.0
        if d >= margin:
            return 0.0
        return (margin - d) / max(1e-9, margin)

    def center_escape(self, x, y, grid_map):
        cx, cy = self.map_width * 0.5, self.map_height * 0.5
        ang = math.atan2(cy - y, cx - x)
        nx, ny = self.try_step_with_avoid(x, y, ang, grid_map)
        if nx is not None:
            return nx, ny, ang
        return None, None, None

    def random_step(self, x, y):
        ang = random.uniform(0, 2 * math.pi)
        return x + self.speed * math.cos(ang), y + self.speed * math.sin(ang)

    def _angle_diff(self, a, b):
        """返回两个角度差值，范围 [-pi, pi]"""
        return (a - b + math.pi) % (2 * math.pi) - math.pi

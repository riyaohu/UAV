import math
from .base_algorithm import BaseAlgorithm

class CoverageSearch(BaseAlgorithm):
    """
    覆盖式搜索（蛇形扫描）
    思路：按固定步长沿着 x 方向走，走到边界就下移一条"扫描线"，再反向走回来。
    """
    uses_belief = False

    def __init__(self, map_width, map_height, step=20, margin=10, lane_spacing=None):
        super().__init__(map_width, map_height)
        self.name = "Coverage Search"
        self.step = step
        self.lane_spacing = lane_spacing if lane_spacing is not None else step
        self.margin = margin
        self.direction = 1        # 1 向右，-1 向左
        self.mode = "scan"        # "scan" or "shift"
        self.shift_remaining = 0
        self.shift_step = step

        # 连续卡住计数：超过阈值就强制跳过当前换行
        self._shift_stuck = 0
        self._shift_stuck_limit = 20  # 最多卡 20 帧就放弃这次换行

    def reset(self):
        self.direction = 1
        self.mode = "scan"
        self.shift_remaining = 0
        self._shift_stuck = 0

    def _is_free(self, x, y, grid_map):
        if grid_map is None:
            return True
        return not grid_map.is_blocked_px(x, y)

    def get_next_position(self, current_x, current_y, current_angle, **kwargs):
        grid_map = kwargs.get("grid_map", None)

        # ====== 换行模式 ======
        if self.mode == "shift":
            dy = min(self.shift_step, self.shift_remaining)
            candidate_x = current_x
            candidate_y = current_y + dy

            moved = False
            if self._is_free(candidate_x, candidate_y, grid_map):
                moved = True
            elif self._is_free(candidate_x + self.step, candidate_y, grid_map):
                candidate_x += self.step
                moved = True
            elif self._is_free(candidate_x - self.step, candidate_y, grid_map):
                candidate_x -= self.step
                moved = True

            if moved:
                # 正常下移
                self.shift_remaining -= dy
                self._shift_stuck = 0
                if self.shift_remaining <= 0:
                    self.mode = "scan"
                return candidate_x, candidate_y
            else:
                # 下方被障碍完全挡住：计卡住帧数
                self._shift_stuck += 1
                if self._shift_stuck >= self._shift_stuck_limit:
                    # 放弃这次换行，强制退出 shift 模式，方向反转继续扫描
                    self.mode = "scan"
                    self.shift_remaining = 0
                    self._shift_stuck = 0
                    self.direction *= -1  # 反向，避免一直往障碍方向扫
                # 本帧原地不动
                return current_x, current_y

        # ====== 扫描模式 ======
        next_x = current_x + self.direction * self.step
        next_y = current_y

        # 触边：进入换行模式
        if next_x > self.map_width - self.margin:
            next_x = self.map_width - self.margin
            self.direction = -1
            self.mode = "shift"
            self.shift_remaining = self.lane_spacing
            self._shift_stuck = 0

        elif next_x < self.margin:
            next_x = self.margin
            self.direction = 1
            self.mode = "shift"
            self.shift_remaining = self.lane_spacing
            self._shift_stuck = 0

        # y 超界：一轮扫描结束，重置回顶部重新开始（循环覆盖，应对漏检）
        if next_y > self.map_height - self.margin:
            next_y = self.margin
            self.direction = 1  # 每轮从左往右重新开始

        # 障碍规避
        if self._is_free(next_x, next_y, grid_map):
            return next_x, next_y

        # 尝试跳过一条扫描线
        alt_y = next_y + self.lane_spacing
        if alt_y <= self.map_height - self.margin and self._is_free(next_x, alt_y, grid_map):
            return next_x, alt_y

        # 尝试反向
        rev_x = current_x + (-self.direction) * self.step
        rev_x = min(max(rev_x, self.margin), self.map_width - self.margin)
        if self._is_free(rev_x, next_y, grid_map):
            self.direction *= -1
            return rev_x, next_y

        # 实在不行，原地等（simulator 的 on_obstacle_hit 会处理）
        return current_x, current_y
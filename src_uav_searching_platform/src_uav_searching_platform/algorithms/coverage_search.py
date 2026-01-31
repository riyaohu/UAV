import math
from .base_algorithm import BaseAlgorithm

class CoverageSearch(BaseAlgorithm):
    """
    覆盖式搜索（蛇形扫描）
    思路：按固定步长沿着 x 方向走，走到边界就下移一条“扫描线”，再反向走回来。
    """
    uses_belief = False

    def __init__(self, map_width, map_height, step=20, margin=10, lane_spacing=None):

        super().__init__(map_width, map_height)
        self.name = "Coverage Search"
        self.step = step          # 每次前进的像素
        self.lane_spacing = lane_spacing if lane_spacing is not None else step
        self.margin = margin      # 离边界留一点，避免越界
        self.direction = 1        # 1 表示向右，-1 表示向左
        # ===== 换行状态机（解决“换行被限速缩小导致轨迹重合”）=====
        self.mode = "scan"          # "scan" or "shift"
        self.shift_remaining = 0    # 还需要向下移动多少像素
        self.shift_step = step      # 每帧向下移动的像素（跟水平步长一致）

    def reset(self):
        self.direction = 1
        self.mode = "scan"
        self.shift_remaining = 0

    def _is_free(self, x, y, grid_map):
        """判断像素坐标 (x,y) 是否可通行。无 grid_map 时默认可通行。"""
        if grid_map is None:
            return True
        # grid_map.is_blocked_px(x, y) True 表示障碍/不可通行
        return (not grid_map.is_blocked_px(x, y))

    def get_next_position(self, current_x, current_y, current_angle, **kwargs):
        # 目标：沿 x 扫描；到边界则 y += step，方向反转
        grid_map = kwargs.get("grid_map", None)
        # ====== 如果正在“换行”，优先执行换行（多帧下移）======
        if self.mode == "shift":
            dy = min(self.shift_step, self.shift_remaining)
            candidate_x = current_x
            candidate_y = current_y + dy

            # 下移也要避障：如果正下方是障碍，尝试左右微移一点点再下移
            if not self._is_free(candidate_x, candidate_y, grid_map):
                # 先尝试向右挪一小步
                if self._is_free(candidate_x + self.step, candidate_y, grid_map):
                    candidate_x += self.step
                # 再尝试向左挪一小步
                elif self._is_free(candidate_x - self.step, candidate_y, grid_map):
                    candidate_x -= self.step
                else:
                    # 实在不行：这一帧先不下移，避免卡死
                    return current_x, current_y

            self.shift_remaining -= dy
            if self.shift_remaining <= 0:
                self.mode = "scan"

            return candidate_x, candidate_y

        next_x = current_x + self.direction * self.step
        next_y = current_y

        # 触边判定：到边界时，不直接跳到下一行，而是进入 shift 模式
        if next_x > self.map_width - self.margin:
            next_x = self.map_width - self.margin
            self.direction = -1
            self.mode = "shift"
            self.shift_remaining = self.lane_spacing

        elif next_x < self.margin:
            next_x = self.margin
            self.direction = 1
            self.mode = "shift"
            self.shift_remaining = self.lane_spacing

        # y 超界：从顶部重新开始（先跑通，后续可改成终止/换策略）
        if next_y > self.map_height - self.margin:
            next_y = self.margin

        # ====== 障碍规避（最小版本）======
        # 方案1：原计划点可走，直接走
        if self._is_free(next_x, next_y, grid_map):
            return next_x, next_y

        # 方案2：尝试向下挪一条扫描线（保持扫描结构）
        alt_x, alt_y = next_x, next_y + + self.lane_spacing
        if alt_y <= self.map_height - self.margin and self._is_free(alt_x, alt_y, grid_map):
            return alt_x, alt_y

        # 方案3：尝试反向走一步（换方向）
        rev_dir = -self.direction
        alt_x, alt_y = current_x + rev_dir * self.step, current_y
        # 边界夹紧
        alt_x = min(max(alt_x, self.margin), self.map_width - self.margin)
        if self._is_free(alt_x, alt_y, grid_map):
            self.direction = rev_dir
            return alt_x, alt_y

        # 方案4：实在不行，原地不动（交给 Simulator 的防卡机制）
        return current_x, current_y


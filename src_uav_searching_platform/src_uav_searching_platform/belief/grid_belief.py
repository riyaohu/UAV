# src_uav_searching_platform/belief/grid_belief.py

import math

class GridBelief:
    """
    一个最简版的置信栅格：
    - 用 grid[r][c] 存概率
    - 只对 free cell 分配概率（障碍格子概率恒为0）
    """

    def __init__(self, grid_map):
        self.grid_map = grid_map
        self.rows = grid_map.rows
        self.cols = grid_map.cols
        self.cell_size = grid_map.cell_size

        # belief[r][c] = probability
        self.belief = [[0.0 for _ in range(self.cols)] for _ in range(self.rows)]
        self.reset()

    def reset(self):
        """均匀分布在所有 free cell 上"""
        free_cells = []
        for r in range(self.rows):
            for c in range(self.cols):
                if self.grid_map.grid[r][c] == 0:  # 0=free, 1=blocked
                    free_cells.append((r, c))

        if not free_cells:
            return

        p = 1.0 / len(free_cells)
        for r, c in free_cells:
            self.belief[r][c] = p

    def px_to_cell(self, x, y):
        """像素坐标 -> 栅格坐标"""
        c = int(x // self.cell_size)
        r = int(y // self.cell_size)
        if r < 0 or r >= self.rows or c < 0 or c >= self.cols:
            return None
        return (r, c)

    def cell_center_px(self, r, c):
        """栅格中心点像素坐标"""
        cx = c * self.cell_size + self.cell_size / 2
        cy = r * self.cell_size + self.cell_size / 2
        return cx, cy

    def _renormalize(self):
        """归一化，让所有概率和=1（只对free cell）"""
        s = 0.0
        for r in range(self.rows):
            for c in range(self.cols):
                s += self.belief[r][c]

        if s <= 1e-12:
            # 如果全被压到0了，就重新均匀
            self.reset()
            return

        for r in range(self.rows):
            for c in range(self.cols):
                self.belief[r][c] /= s

    def update_negative(self, uav_x, uav_y, radius_px, decay=0.5):
        """
        负证据更新：
        - UAV 在半径 radius_px 的范围内没有检测到目标
        - 那么这个范围内格子的概率降低（乘以 decay，0<decay<1）
        """
        r0c0 = self.px_to_cell(uav_x, uav_y)
        if r0c0 is None:
            return

        for r in range(self.rows):
            for c in range(self.cols):
                if self.grid_map.grid[r][c] == 1:
                    continue  # 障碍格子不更新
                cx, cy = self.cell_center_px(r, c)
                d = math.hypot(cx - uav_x, cy - uav_y)
                if d <= radius_px:
                    self.belief[r][c] *= decay

        self._renormalize()

    def mark_found(self, x, y):
        """目标被找到：把这个cell概率清零（表示该位置已解释/不再关注）"""
        cell = self.px_to_cell(x, y)
        if cell is None:
            return
        r, c = cell
        self.belief[r][c] = 0.0
        self._renormalize()

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
        # 【新增】探索阈值：小于它认为“已探索”
        self.explored_threshold = 0.001

        # belief[r][c] = probability
        self.belief = [[0.0 for _ in range(self.cols)] for _ in range(self.rows)]

        # explored[r][c] = 0/1，表示是否被“看过/扫描过”
        self.explored = [[0.0 for _ in range(self.cols)] for _ in range(self.rows)]

        # frontier 用的阈值（>= 这个值就当作 explored）
        self.explored_threshold = 0.5

        # 兼容你之前 frontier_search 写法：belief.grid -> belief.belief
        self.grid = self.belief

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

        for r in range(self.rows):
            for c in range(self.cols):
                self.explored[r][c] = 0.0
        # --- [ADD] baseline probabilities for thresholding ---
        self.free_cells_count = sum(
            1 for r in range(self.rows) for c in range(self.cols)
            if not self.grid_map.is_blocked_cell(r, c)
        )
        self.uniform_p = 1.0 / max(1, self.free_cells_count)

        # “探索过”：概率显著低于均匀分布
        self.explored_threshold = self.uniform_p * 0.6
        # “未探索”：概率显著高于均匀分布（还没怎么被“压低”）
        self.unexplored_threshold = self.uniform_p * 1.2

    def is_explored(self, r, c):
        return self.explored[r][c] >= 0.5

    def is_unexplored(self, r, c):
        return self.explored[r][c] < 0.5

    def mark_explored_in_radius(self, uav_x, uav_y, radius_px):
        """把 UAV 半径内的格子标记为 explored=1（不做归一化）"""
        if self.grid_map is None:
            return

        # 像素 -> 格子范围（减少遍历）
        r0, c0 = self.grid_map.px_to_cell(uav_x, uav_y)
        rad_cells = int(radius_px / self.cell_size) + 1

        r_min = max(0, r0 - rad_cells)
        r_max = min(self.rows - 1, r0 + rad_cells)
        c_min = max(0, c0 - rad_cells)
        c_max = min(self.cols - 1, c0 + rad_cells)

        rr2 = radius_px * radius_px
        for r in range(r_min, r_max + 1):
            for c in range(c_min, c_max + 1):
                if self.grid_map.is_blocked_cell(r, c):
                    continue
                px, py = self.cell_center_px(r, c)
                dx = px - uav_x
                dy = py - uav_y
                if dx * dx + dy * dy <= rr2:
                    self.explored[r][c] = 1.0

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
    # --- compatibility aliases (旧代码兼容) ---
    def pixel_to_cell(self, x, y):
        return self.px_to_cell(x, y)

    def cell_to_pixel(self, r, c):
        return self.cell_center_px(r, c)

    def cell_to_px(self, r, c):
        return self.cell_center_px(r, c)


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
        负证据更新（半径内更新版）：
        - UAV 在半径 radius_px 的范围内没有检测到目标
        - 那么这个范围内格子的概率降低（乘以 decay，0<decay<1）

        优化点：
        - 不再遍历全图，只遍历“可能落在圆内”的小方块区域，再用距离筛选成圆。
        """
        r0c0 = self.px_to_cell(uav_x, uav_y)
        if r0c0 is None:
            return
        r0, c0 = r0c0

        # 1) 把“像素半径”换算成“格子半径”（往上取整，确保覆盖到圆）
        k = int(math.ceil(radius_px / self.cell_size))

        # 2) 只遍历 UAV 周围 (2k+1)x(2k+1) 的小区域（边界裁剪）
        r_min = max(0, r0 - k)
        r_max = min(self.rows - 1, r0 + k)
        c_min = max(0, c0 - k)
        c_max = min(self.cols - 1, c0 + k)

        radius2 = radius_px * radius_px
        cs = self.cell_size

        for r in range(r_min, r_max + 1):
            # 直接算格子中心点的像素 y（避免函数调用开销）
            cy = (r + 0.5) * cs
            dy = cy - uav_y
            for c in range(c_min, c_max + 1):
                if self.grid_map.grid[r][c] == 1:
                    continue  # 障碍格子不更新

                cx = (c + 0.5) * cs
                dx = cx - uav_x

                # 3) 用平方距离判断是否在圆内（比 hypot 更快，结果一样）
                if dx * dx + dy * dy <= radius2:
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

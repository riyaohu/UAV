# src_uav_searching_platform/environment/grid_map.py
import random

class GridMap:
    def __init__(self, width_px, height_px, cell_size, obstacle_density=0.1, border_blocked=False):
        self.width_px = width_px
        self.height_px = height_px
        self.cell_size = cell_size

        self.cols = width_px // cell_size
        self.rows = height_px // cell_size

        # grid[r][c] = 1 表示障碍，0 表示可通行
        self.grid = [[0 for _ in range(self.cols)] for _ in range(self.rows)]

        # 生成障碍
        for r in range(self.rows):
            for c in range(self.cols):
                if border_blocked and (r == 0 or c == 0 or r == self.rows - 1 or c == self.cols - 1):
                    self.grid[r][c] = 1
                else:
                    self.grid[r][c] = 1 if random.random() < obstacle_density else 0

    def px_to_cell(self, x_px, y_px):
        c = int(x_px // self.cell_size)
        r = int(y_px // self.cell_size)
        return r, c

    def in_bounds_cell(self, r, c):
        return 0 <= r < self.rows and 0 <= c < self.cols

    def is_blocked_cell(self, r, c):
        if not self.in_bounds_cell(r, c):
            return True  # 出界当作障碍
        return self.grid[r][c] == 1

    def is_blocked_px(self, x_px, y_px):
        r, c = self.px_to_cell(x_px, y_px)
        return self.is_blocked_cell(r, c)

    def random_free_position(self):
        # 简单暴力采样：找到一个可通行格
        while True:
            r = random.randrange(self.rows)
            c = random.randrange(self.cols)
            if self.grid[r][c] == 0:
                # 返回该格中心点像素坐标
                x = (c + 0.5) * self.cell_size
                y = (r + 0.5) * self.cell_size
                return x, y

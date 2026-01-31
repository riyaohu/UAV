import math
from .base_algorithm import BaseAlgorithm

class FrontierSearch(BaseAlgorithm):
    name = "Frontier Search"
    uses_belief = True   # 关键：这个算法依赖 belief

    def __init__(self, map_width, map_height, speed=3.0, gain_radius_cells=4, lambda_cost=0.002):
        super().__init__(map_width, map_height)
        self.speed = speed

        # --- [B] parameters ---
        self.gain_radius_cells = gain_radius_cells  # 计算“附近信息量”的半径（格子）
        self.lambda_cost = lambda_cost  # 距离代价权重（越大越偏向近）

    def get_next_position(self, current_x, current_y, current_angle, **kwargs):
        belief = kwargs.get("belief", None)
        # 兜底：如果没有 belief，就随机走
        if belief is None:
            return self.random_step(current_x, current_y)

        grid_map = kwargs.get("grid_map", None)

        frontier_cells = self.find_frontiers(belief)

        if not frontier_cells:
            # 没有 frontier：先随便走两步，把 explored 扩出去
            return self.random_step(current_x, current_y)

        """
        uav_x, uav_y: 当前 UAV 像素坐标
        belief: GridBelief 对象
        grid_map: GridMap（用于像素-格子转换）
        """

        # 1. 找所有 frontier 格子


        best = None
        best_score = -1e18

        uav_r, uav_c = belief.grid_map.px_to_cell(current_x, current_y)

        for (r, c) in frontier_cells:
            gain = self.frontier_gain(belief, r, c)

            # cost 用格子距离近似即可
            dr = (r - uav_r)
            dc = (c - uav_c)
            cost = (dr * dr + dc * dc) ** 0.5

            score = gain - self.lambda_cost * cost

            if score > best_score:
                best_score = score
                best = (r, c)

        if best is None:
            return current_x, current_y

        target_x, target_y = belief.grid_map.cell_center_px(best[0], best[1])
        return self.move_towards_avoid(current_x, current_y, target_x, target_y, belief.grid_map)

    def find_frontiers(self, belief):
        frontiers = []
        for r in range(1, belief.rows - 1):
            for c in range(1, belief.cols - 1):
                if belief.grid_map.is_blocked_cell(r, c):
                    continue

                # explored cell
                if not belief.is_explored(r, c):
                    continue

                # has any unexplored neighbor
                neighbors = [(r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)]
                for rr, cc in neighbors:
                    if belief.grid_map.is_blocked_cell(rr, cc):
                        continue
                    if belief.is_unexplored(rr, cc):
                        frontiers.append((r, c))
                        break
        return frontiers

    def frontier_gain(self, belief, r, c):
        """用“附近还剩多少概率质量/未探索程度”近似 gain。"""
        R = self.gain_radius_cells
        s = 0.0
        for rr in range(max(0, r - R), min(belief.rows, r + R + 1)):
            for cc in range(max(0, c - R), min(belief.cols, c + R + 1)):
                if belief.grid_map.is_blocked_cell(rr, cc):
                    continue
                s += belief.belief[rr][cc]
        return s

    def find_nearest_frontier(self, uav_x, uav_y, frontiers, belief):
        min_dist = float("inf")
        best = frontiers[0]

        for r, c in frontiers:
            px, py = belief.cell_to_pixel(r, c)
            d = math.hypot(px - uav_x, py - uav_y)
            if d < min_dist:
                min_dist = d
                best = (r, c)
        return best

    def move_towards(self, x, y, tx, ty):
        dx = tx - x
        dy = ty - y
        dist = math.hypot(dx, dy)
        if dist == 0:
            return x, y
        step_x = x + self.speed * dx / dist
        step_y = y + self.speed * dy / dist
        return step_x, step_y

    def random_step(self, x, y):
        import random
        angle = random.uniform(0, 2 * math.pi)
        return (
            x + self.speed * math.cos(angle),
            y + self.speed * math.sin(angle),
        )

    def move_towards_avoid(self, x, y, tx, ty, grid_map):
        """
        先按目标方向走一步；如果这一步会撞障碍，就在目标方向附近“扇形试探”找一个可走的方向。
        """
        dx = tx - x
        dy = ty - y
        dist = math.hypot(dx, dy)
        if dist == 0:
            return x, y

        # 1) 目标方向的一步
        step_x = x + self.speed * dx / dist
        step_y = y + self.speed * dy / dist
        if not grid_map.is_blocked_px(step_x, step_y):
            return step_x, step_y

        # 2) 撞障碍：在目标方向附近尝试多个偏转角
        base = math.atan2(dy, dx)
        delta = math.pi / 12  # 15度
        for k in range(1, 13):  # 最多试到 180 度
            for sgn in (+1, -1):
                ang = base + sgn * k * delta
                cand_x = x + self.speed * math.cos(ang)
                cand_y = y + self.speed * math.sin(ang)
                if not grid_map.is_blocked_px(cand_x, cand_y):
                    return cand_x, cand_y

        # 3) 实在走不了：随机找几个方向，找不到就原地
        import random
        for _ in range(20):
            ang = random.uniform(0, 2 * math.pi)
            cand_x = x + self.speed * math.cos(ang)
            cand_y = y + self.speed * math.sin(ang)
            if not grid_map.is_blocked_px(cand_x, cand_y):
                return cand_x, cand_y

        return x, y

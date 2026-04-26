"""
Random Search Algorithm - baseline algorithm
"""
import random
import math
from .base_algorithm import BaseAlgorithm
from .. import config


class RandomSearch(BaseAlgorithm):
    """Random search algorithm with smooth movement"""
    uses_belief = False

    def __init__(self, map_width, map_height):
        super().__init__(map_width, map_height)
        self.name = "Random Search"
        self.current_direction = random.uniform(0, 360)
        self.turn_probability = config.RANDOM_SEARCH_TURN_PROBABILITY
        self.max_turn_angle = config.RANDOM_SEARCH_MAX_TURN_ANGLE
        self.speed = config.UAV_SPEED

        # 撞障碍后连续卡住的帧计数，用于触发逃脱策略
        self._stuck_frames = 0

    def get_next_position(self, current_x, current_y, current_angle, **kwargs):
        # Randomly decide whether to change direction
        if random.random() < self.turn_probability:
            turn_angle = random.uniform(-self.max_turn_angle, self.max_turn_angle)
            self.current_direction += turn_angle
            self.current_direction %= 360

        # Calculate next position based on current direction
        rad = math.radians(self.current_direction)
        dx = self.speed * math.cos(rad)
        dy = self.speed * math.sin(rad)

        next_x = current_x + dx
        next_y = current_y + dy

        # Boundary checking and bouncing
        bounced = False
        if next_x < 0 or next_x > self.map_width:
            self.current_direction = 180 - self.current_direction
            self.current_direction %= 360
            next_x = max(0, min(self.map_width, next_x))
            bounced = True

        if next_y < 0 or next_y > self.map_height:
            self.current_direction = -self.current_direction
            self.current_direction %= 360
            next_y = max(0, min(self.map_height, next_y))
            bounced = True

        if bounced:
            rad = math.radians(self.current_direction)
            dx = self.speed * math.cos(rad)
            dy = self.speed * math.sin(rad)
            next_x = current_x + dx
            next_y = current_y + dy
            next_x = max(0, min(self.map_width, next_x))
            next_y = max(0, min(self.map_height, next_y))

        return next_x, next_y

    def on_obstacle_hit(self, current_x, current_y, grid_map=None):
        """
        撞到障碍物时调用（由 simulator 触发）。
        策略：系统遍历 N 个候选方向，选第一个能走的；
        若全堵死则随机选一个方向（极端情况）。
        相比 reset() 每次完全随机，这里保证当帧就能脱困。
        """
        self._stuck_frames += 1

        # 生成候选方向：在当前方向基础上，均匀撒点 + 随机扰动
        num_candidates = 16
        candidates = []
        for i in range(num_candidates):
            angle = (self.current_direction + 180 + i * (360 / num_candidates)) % 360
            candidates.append(angle)
        # 再加几个完全随机方向兜底
        for _ in range(4):
            candidates.append(random.uniform(0, 360))

        if grid_map is not None:
            for angle in candidates:
                rad = math.radians(angle)
                nx = current_x + self.speed * math.cos(rad)
                ny = current_y + self.speed * math.sin(rad)
                nx = max(0, min(self.map_width, nx))
                ny = max(0, min(self.map_height, ny))
                if not grid_map.is_blocked_px(nx, ny):
                    self.current_direction = angle
                    self._stuck_frames = 0
                    return
            # 所有方向都堵死（极端角落），随机选一个
            self.current_direction = random.uniform(0, 360)
        else:
            # 无栅格地图时退化为原来行为：反向 + 随机扰动
            self.current_direction = (self.current_direction + 180 + random.uniform(-30, 30)) % 360

        self._stuck_frames = 0

    def reset(self):
        """Reset algorithm state (called on full simulation reset)"""
        self.current_direction = random.uniform(0, 360)
        self._stuck_frames = 0
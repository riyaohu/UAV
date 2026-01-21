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
        """
        Initialize random search algorithm

        Args:
            map_width: Width of the search map
            map_height: Height of the search map
        """
        super().__init__(map_width, map_height)
        self.name = "Random Search"
        self.current_direction = random.uniform(0, 360)  # Initial random direction
        self.turn_probability = config.RANDOM_SEARCH_TURN_PROBABILITY
        self.max_turn_angle = config.RANDOM_SEARCH_MAX_TURN_ANGLE
        self.speed = config.UAV_SPEED

    def get_next_position(self, current_x, current_y, current_angle, **kwargs):
        """
        Calculate next position using random search with smooth turns

        Args:
            current_x: Current x coordinate
            current_y: Current y coordinate
            current_angle: Current heading angle (not used much, we use internal direction)
            **kwargs: Additional parameters

        Returns:
            tuple: (next_x, next_y) coordinates
        """
        # Randomly decide whether to change direction
        if random.random() < self.turn_probability:
            # Random turn within max_turn_angle range
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
            # Bounce off left/right wall
            self.current_direction = 180 - self.current_direction
            self.current_direction %= 360
            next_x = max(0, min(self.map_width, next_x))
            bounced = True

        if next_y < 0 or next_y > self.map_height:
            # Bounce off top/bottom wall
            self.current_direction = -self.current_direction
            self.current_direction %= 360
            next_y = max(0, min(self.map_height, next_y))
            bounced = True

        # Recalculate position after bounce
        if bounced:
            rad = math.radians(self.current_direction)
            dx = self.speed * math.cos(rad)
            dy = self.speed * math.sin(rad)
            next_x = current_x + dx
            next_y = current_y + dy
            # Clamp to boundaries
            next_x = max(0, min(self.map_width, next_x))
            next_y = max(0, min(self.map_height, next_y))

        return next_x, next_y

    def reset(self):
        """Reset algorithm state"""
        self.current_direction = random.uniform(0, 360)



"""
Base class for search algorithms
"""
from abc import ABC, abstractmethod


class BaseAlgorithm(ABC):
    """Base class for all search algorithms"""
    uses_belief = False  # 默认：不需要 belief
    def __init__(self, map_width, map_height):
        """
        Initialize the algorithm

        Args:
            map_width: Width of the search map
            map_height: Height of the search map
        """
        self.map_width = map_width
        self.map_height = map_height
        self.name = "Base Algorithm"

    @abstractmethod
    def get_next_position(self, current_x, current_y, current_angle, **kwargs):
        """
        Calculate the next position for the UAV

        Args:
            current_x: Current x coordinate
            current_y: Current y coordinate
            current_angle: Current heading angle in degrees
            **kwargs: Additional parameters specific to the algorithm

        Returns:
            tuple: (next_x, next_y) coordinates
        """
        pass

    def reset(self):
        """Reset algorithm state (override if needed)"""
        pass

    def get_name(self):
        """
        Get algorithm name

        Returns:
            str: Algorithm name
        """
        return self.name



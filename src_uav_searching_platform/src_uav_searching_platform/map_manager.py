"""
Map Manager - handles map loading and rendering
"""
import pygame
from . import config



class MapManager:
    """Manages the map background"""

    def __init__(self, map_image_path=None):
        """
        Initialize the map manager

        Args:
            map_image_path: Path to the map image file
        """
        if map_image_path is None:
            map_image_path = config.MAP_IMAGE_PATH

        self.image_path = map_image_path
        self.image = None
        self.width = config.WINDOW_WIDTH
        self.height = config.WINDOW_HEIGHT

        self.load_image()

    def load_image(self):
        """Load and scale the map image"""
        try:
            # Load the original image
            original_image = pygame.image.load(self.image_path)
            # Scale to fit the window
            self.image = pygame.transform.scale(
                original_image,
                (self.width, self.height)
            )
            print(f"Map loaded successfully: {self.image_path}")
        except pygame.error as e:
            print(f"Failed to load map image: {e}")
            # Create a default background if image loading fails
            self.image = pygame.Surface((self.width, self.height))
            self.image.fill(config.COLOR_WHITE)

    def draw(self, screen):
        """
        Draw the map on the screen

        Args:
            screen: Pygame screen surface
        """
        screen.blit(self.image, (0, 0))

    def is_within_bounds(self, x, y):
        """
        Check if a position is within map boundaries

        Args:
            x: X coordinate
            y: Y coordinate

        Returns:
            bool: True if within bounds, False otherwise
        """
        return 0 <= x <= self.width and 0 <= y <= self.height

    def get_size(self):
        """
        Get map dimensions

        Returns:
            tuple: (width, height)
        """
        return self.width, self.height

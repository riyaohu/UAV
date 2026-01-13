"""
Target class - represents the search target
"""
import pygame
import math
from . import config



class Target:
    """Represents a target to be searched for"""

    def __init__(self, x, y, image_path=None):
        """
        Initialize the target

        Args:
            x: X coordinate
            y: Y coordinate
            image_path: Path to target image file
        """
        self.x = x
        self.y = y
        self.found = False
        self.image = None
        self.size = config.TARGET_SIZE

        if image_path is None:
            image_path = config.TARGET_IMAGE_PATH

        self.load_image(image_path)

    def load_image(self, image_path):
        """
        Load and scale the target image

        Args:
            image_path: Path to the image file
        """
        try:
            original_image = pygame.image.load(image_path)
            self.image = pygame.transform.scale(
                original_image,
                (self.size, self.size)
            )
            print(f"Target image loaded: {image_path}")
        except pygame.error as e:
            print(f"Failed to load target image: {e}")
            # Create a default circle if image loading fails
            self.image = pygame.Surface((self.size, self.size), pygame.SRCALPHA)
            pygame.draw.circle(
                self.image,
                config.COLOR_RED,
                (self.size // 2, self.size // 2),
                self.size // 2
            )

    def is_detected(self, uav_x, uav_y, detection_radius):
        """
        Check if the target is detected by UAV

        Args:
            uav_x: UAV x coordinate
            uav_y: UAV y coordinate
            detection_radius: Detection range

        Returns:
            bool: True if detected, False otherwise
        """
        distance = math.sqrt((self.x - uav_x) ** 2 + (self.y - uav_y) ** 2)
        if distance <= detection_radius and not self.found:
            self.found = True
            return True
        return False

    def draw(self, screen):
        """
        Draw the target on screen

        Args:
            screen: Pygame screen surface
        """
        # Calculate position to center the image
        draw_x = self.x - self.size // 2
        draw_y = self.y - self.size // 2

        # Draw the target
        screen.blit(self.image, (draw_x, draw_y))

        # If found, draw a green circle around it
        if self.found:
            pygame.draw.circle(
                screen,
                config.COLOR_GREEN,
                (int(self.x), int(self.y)),
                self.size // 2 + 5,
                3
            )

    def reset(self):
        """Reset target found status"""
        self.found = False

    def get_position(self):
        """
        Get target position

        Returns:
            tuple: (x, y) coordinates
        """
        return self.x, self.y

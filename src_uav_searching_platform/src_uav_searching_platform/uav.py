"""
UAV class - represents the unmanned aerial vehicle
"""
import pygame
import math
import config


class UAV:
    """Represents an unmanned aerial vehicle (UAV)"""

    def __init__(self, x, y, image_path=None):
        """
        Initialize the UAV

        Args:
            x: Starting x coordinate
            y: Starting y coordinate
            image_path: Path to UAV image file
        """
        self.start_x = x
        self.start_y = y
        self.x = x
        self.y = y
        self.speed = config.UAV_SPEED
        self.detection_radius = config.UAV_DETECTION_RADIUS
        self.angle = 0  # heading angle in degrees (0 = right, 90 = down)
        self.size = config.UAV_SIZE

        # Trajectory tracking
        self.trajectory = [(x, y)]
        self.max_trajectory_points = config.MAX_TRAJECTORY_POINTS

        # Statistics
        self.distance_traveled = 0.0
        self.time_elapsed = 0  # in frames

        # Load image
        self.original_image = None
        self.image = None
        if image_path is None:
            image_path = config.UAV_IMAGE_PATH
        self.load_image(image_path)

    def load_image(self, image_path):
        """
        Load and scale the UAV image

        Args:
            image_path: Path to the image file
        """
        try:
            loaded_image = pygame.image.load(image_path)
            self.original_image = pygame.transform.scale(
                loaded_image,
                (self.size, self.size)
            )
            self.image = self.original_image.copy()
            print(f"UAV image loaded: {image_path}")
        except pygame.error as e:
            print(f"Failed to load UAV image: {e}")
            # Create a default triangle if image loading fails
            self.original_image = pygame.Surface((self.size, self.size), pygame.SRCALPHA)
            points = [
                (self.size, self.size // 2),
                (0, 0),
                (0, self.size)
            ]
            pygame.draw.polygon(self.original_image, config.COLOR_BLUE, points)
            self.image = self.original_image.copy()

    def move(self, dx, dy, map_width, map_height):
        """
        Move the UAV by the given displacement

        Args:
            dx: Change in x
            dy: Change in y
            map_width: Map width for boundary checking
            map_height: Map height for boundary checking
        """
        new_x = self.x + dx
        new_y = self.y + dy

        # Boundary checking
        if 0 <= new_x <= map_width and 0 <= new_y <= map_height:
            # Calculate distance
            distance = math.sqrt(dx ** 2 + dy ** 2)
            self.distance_traveled += distance

            # Update position
            self.x = new_x
            self.y = new_y

            # Update angle based on movement direction
            if dx != 0 or dy != 0:
                self.angle = math.degrees(math.atan2(dy, dx))

            # Update trajectory
            self.trajectory.append((self.x, self.y))
            if len(self.trajectory) > self.max_trajectory_points:
                self.trajectory.pop(0)

        self.time_elapsed += 1

    def set_position(self, x, y):
        """
        Set UAV position directly (for algorithm control)

        Args:
            x: New x coordinate
            y: New y coordinate
        """
        dx = x - self.x
        dy = y - self.y
        self.x = x
        self.y = y

        # Update angle
        if dx != 0 or dy != 0:
            self.angle = math.degrees(math.atan2(dy, dx))

        # Update distance
        distance = math.sqrt(dx ** 2 + dy ** 2)
        self.distance_traveled += distance

        # Update trajectory
        self.trajectory.append((self.x, self.y))
        if len(self.trajectory) > self.max_trajectory_points:
            self.trajectory.pop(0)

        self.time_elapsed += 1

    def draw(self, screen, show_detection_range=True, show_trajectory=True):
        """
        Draw the UAV on screen

        Args:
            screen: Pygame screen surface
            show_detection_range: Whether to show detection circle
            show_trajectory: Whether to show movement trajectory
        """
        # Draw detection range
        if show_detection_range:
            surface = pygame.Surface((screen.get_width(), screen.get_height()), pygame.SRCALPHA)
            pygame.draw.circle(
                surface,
                config.COLOR_DETECTION_RANGE,
                (int(self.x), int(self.y)),
                self.detection_radius
            )
            screen.blit(surface, (0, 0))

            # Draw detection range outline
            pygame.draw.circle(
                screen,
                config.COLOR_GREEN,
                (int(self.x), int(self.y)),
                self.detection_radius,
                1
            )

        # Draw trajectory
        if show_trajectory and len(self.trajectory) > 1:
            pygame.draw.lines(
                screen,
                config.TRAJECTORY_COLOR,
                False,
                [(int(p[0]), int(p[1])) for p in self.trajectory],
                config.TRAJECTORY_WIDTH
            )

        # Rotate image based on heading angle
        rotated_image = pygame.transform.rotate(self.original_image, -self.angle)
        rect = rotated_image.get_rect(center=(self.x, self.y))

        # Draw the UAV
        screen.blit(rotated_image, rect)

    def reset(self):
        """Reset UAV to starting position"""
        self.x = self.start_x
        self.y = self.start_y
        self.angle = 0
        self.trajectory = [(self.x, self.y)]
        self.distance_traveled = 0.0
        self.time_elapsed = 0

    def get_position(self):
        """
        Get UAV position

        Returns:
            tuple: (x, y) coordinates
        """
        return self.x, self.y

    def get_stats(self):
        """
        Get UAV statistics

        Returns:
            dict: Statistics including distance and time
        """
        return {
            'distance': self.distance_traveled,
            'time': self.time_elapsed,
            'position': (self.x, self.y)
        }

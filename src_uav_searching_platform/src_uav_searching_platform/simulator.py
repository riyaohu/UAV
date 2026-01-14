"""
Simulator - main simulation engine
"""
import pygame
from . import config
from .map_manager import MapManager
from .uav import UAV
from .target import Target
from .algorithms import RandomSearch



class Simulator:
    """Main simulator class that manages the simulation"""

    def __init__(self,render=True, mode="experiment"):
        """Initialize the simulator"""
        # Initialize pygame
        self.render = render
        self.mode = mode
        self.stop_reason = None
        if self.render:
            pygame.init()

            # Create window
            self.screen = pygame.display.set_mode((config.WINDOW_WIDTH, config.WINDOW_HEIGHT))
            pygame.display.set_caption(config.WINDOW_TITLE)
            self.clock = pygame.time.Clock()

            # Initialize font
            self.font = pygame.font.Font(None, config.INFO_FONT_SIZE)


        # Create game objects
        self.map_manager = MapManager()
        self.uav = UAV(config.UAV_START_X, config.UAV_START_Y)
        self.targets = []
        for i in range(config.NUM_TARGETS):
            x, y = config.TARGET_POSITIONS[i]
            self.targets.append(Target(x, y))

        # Initialize algorithm
        map_width, map_height = self.map_manager.get_size()
        self.algorithm = RandomSearch(map_width, map_height)

        # Simulation state
        self.running = True
        self.paused = False
        self.target_found = False
        self.search_complete = False

        # Statistics
        self.frames = 0

    def handle_events(self):
        """Handle user input events"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False
                elif event.key == pygame.K_SPACE:
                    self.paused = not self.paused
                elif event.key == pygame.K_r:
                    self.reset()

    def update(self):
        """Update simulation state"""
        if self.paused or self.search_complete:
            return

        # Get next position from algorithm
        current_x, current_y = self.uav.get_position()
        next_x, next_y = self.algorithm.get_next_position(
            current_x, current_y, self.uav.angle
        )

        # Update UAV position
        self.uav.set_position(next_x, next_y)

        # Check if target is detected
        # 逐个检测目标
        for t in self.targets:
            if t.is_detected(self.uav.x, self.uav.y, self.uav.detection_radius):
                print(f"Found one target at ({t.x}, {t.y})! Now: {self.get_found_count()}/{len(self.targets)}")

        # 是否全部找到
        if config.STOP_WHEN_ALL_FOUND:
            if self.get_found_count() == len(self.targets):
                self.target_found = True  # 你可以继续沿用这个变量名：含义改成“全部找到”
                self.search_complete = True
                print(
                    f"All targets found! Time: {self.frames} frames, Distance: {self.uav.distance_traveled:.2f} pixels")

        self.frames += 1

        stop, reason = self.check_termination()
        if stop:
            self.stop_reason = reason
            if self.mode == "experiment":
                self.running = False
            elif self.mode == "demo":
                self.paused = True  # 停下来但不退出

    def get_found_count(self):
        return sum(1 for t in self.targets if t.found)

    def draw(self):
        """Draw all elements on screen"""
        # Draw map background
        self.map_manager.draw(self.screen)

        # Draw target (only if not found, or with highlight if found)
        for t in self.targets:
            t.draw(self.screen)

        # Draw UAV with detection range and trajectory
        self.uav.draw(
            self.screen,
            show_detection_range=config.SHOW_DETECTION_RANGE,
            show_trajectory=config.SHOW_TRAJECTORY
        )

        # Draw info panel
        self.draw_info_panel()

        # Update display
        pygame.display.flip()

    def draw_info_panel(self):
        """Draw information panel with statistics"""
        found_count = self.get_found_count()
        info_lines = [
            f"Algorithm: {self.algorithm.get_name()}",
            f"Time: {self.frames} frames ({self.frames / config.FPS:.1f}s)",
            f"Distance: {self.uav.distance_traveled:.1f} px",
            f"Position: ({int(self.uav.x)}, {int(self.uav.y)})",
            f"Found: {found_count}/{len(self.targets)}",
            f"Status: {'ALL FOUND!' if self.target_found else 'Searching...'}"
        ]

        if self.paused:
            info_lines.append("PAUSED")

        # Calculate panel dimensions
        line_height = config.INFO_FONT_SIZE + 5
        panel_height = len(info_lines) * line_height + 20
        panel_width = 350

        # Draw semi-transparent background
        info_surface = pygame.Surface((panel_width, panel_height), pygame.SRCALPHA)
        info_surface.fill(config.INFO_BG_COLOR)
        self.screen.blit(info_surface, (config.INFO_PANEL_X, config.INFO_PANEL_Y))

        # Draw text lines
        y_offset = config.INFO_PANEL_Y + 10
        for line in info_lines:
            text_surface = self.font.render(line, True, config.INFO_TEXT_COLOR)
            self.screen.blit(text_surface, (config.INFO_PANEL_X + 10, y_offset))
            y_offset += line_height

        # Draw controls help at bottom
        help_lines = [
            "Controls:",
            "SPACE - Pause/Resume",
            "R - Reset",
            "ESC - Quit"
        ]

        help_y = config.WINDOW_HEIGHT - len(help_lines) * line_height - 20
        help_surface = pygame.Surface((panel_width, len(help_lines) * line_height + 20), pygame.SRCALPHA)
        help_surface.fill(config.INFO_BG_COLOR)
        self.screen.blit(help_surface, (config.INFO_PANEL_X, help_y))

        y_offset = help_y + 10
        for line in help_lines:
            text_surface = self.font.render(line, True, config.INFO_TEXT_COLOR)
            self.screen.blit(text_surface, (config.INFO_PANEL_X + 10, y_offset))
            y_offset += line_height

    def reset(self):
        """Reset simulation to initial state"""
        self.uav.reset()
        for t in self.targets:
            t.reset()
        self.algorithm.reset()
        self.paused = False
        self.running = True
        self.target_found = False
        self.search_complete = False
        self.stop_reason = None
        self.frames = 0
        print("Simulation reset")

    def run(self):
        """Main simulation loop"""
        print(f"Starting UAV Search Simulator")
        print(f"Algorithm: {self.algorithm.get_name()}")
        print(f"Targets: {[ (t.x, t.y) for t in self.targets ]}")
        print(f"Detection radius: {self.uav.detection_radius} pixels")
        print("\nControls:")
        print("  SPACE - Pause/Resume")
        print("  R - Reset")
        print("  ESC - Quit")


        while self.running:
            # Handle events
            if self.render:
                self.handle_events()

            # Update simulation
            self.update()

            # Draw everything
            if self.render:
                self.draw()
            # Control frame rate
                self.clock.tick(config.FPS)

        # Cleanup
        if self.render:
            pygame.quit()
        print(f"Stop reason: {self.stop_reason}")
        print(f"Frames: {self.frames}")
        print("Simulator closed")

    def set_algorithm(self, algorithm):
        """
        Change the search algorithm

        Args:
            algorithm: New algorithm instance
        """
        self.algorithm = algorithm
        self.reset()

    # 停止条件
    def check_termination(self):
        if self.target_found:
            return True, "success"

        if self.frames >= config.MAX_FRAMES:
            return True, "timeout"

        if self.uav.distance_traveled >= config.MAX_DISTANCE:
            return True, "distance_limit"

        return False, None


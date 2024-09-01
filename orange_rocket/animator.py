from typing import List
from PIL import Image, ImageDraw

from orange_rocket.optimizer import Obstacle, Result


class Animation:
    def __init__(
        self,
        obstacles: List[Obstacle],
        result: Result,
        frame_size=(800, 600),
        background_color=(0, 0, 0),
    ):
        self.obstacles = obstacles
        self.result = result
        self.frame_size = frame_size
        self.background_color = background_color
        self.frames = []

    def draw_frame(self, vehicle_position):
        frame = Image.new("RGB", self.frame_size, self.background_color)
        draw = ImageDraw.Draw(frame)

        # Draw obstacles
        for obstacle in self.obstacles:
            x, y = obstacle.position
            radius = 10  # Example radius
            draw.ellipse((x - radius, y - radius, x + radius, y + radius), fill="white")

        # Draw vehicle
        x, y = vehicle_position
        vehicle_size = 10  # Example vehicle size
        draw.rectangle(
            (x - vehicle_size, y - vehicle_size, x + vehicle_size, y + vehicle_size),
            fill="orange",
        )

        return frame

    def construct(self):
        for position in self.result.vehicle_positions:
            position = (
                position[0] * self.frame_size[0],
                position[1] * self.frame_size[1],
            )
            frame = self.draw_frame(position)
            self.frames.append(frame)

    def save_animation(self, output_path="animation.mp4", frame_duration=100):
        if not self.frames:
            raise ValueError("No frames to save. Did you call construct()?")

        # Save frames as a GIF
        self.frames[0].save(
            output_path,
            save_all=True,
            append_images=self.frames[1:],
            duration=frame_duration,
            loop=0,
            format="GIF",
        )


if __name__ == "__main__":
    from orange_rocket.optimizer import optimize

    obstacles: List[Obstacle] = [
        # Obstacle(position=(0.5, 0.5), radius=0.1),
        # Obstacle(position=(0.2, 0.8), radius=0.5),
    ]
    result = optimize(
        starting_point=(0, 0.5),
        goal=(1, 1),
        obstacles=obstacles,
        max_time_segments=20,
        thrust_magnitude=0.05,
    )

    animation = Animation(obstacles, result)
    animation.construct()
    animation.save_animation("animation.gif")

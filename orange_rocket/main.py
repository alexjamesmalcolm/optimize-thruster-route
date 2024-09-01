from pprint import pprint
from typing import List

from orange_rocket.animator import Animation
from orange_rocket.optimizer import Obstacle, optimize


if __name__ == "__main__":
    print("Running orange_rocket")
    obstacles: List[Obstacle] = [
        # Obstacle(position=(0.5, 0.5), radius=0.1),
        # Obstacle(position=(0.2, 0.8), radius=0.5),
    ]
    result = optimize(
        starting_point=(0, 0.5),
        goal=(1, 1),
        obstacles=obstacles,
        max_time_segments=100,
        thrust_magnitude=0.001,
    )
    pprint(result)
    animation = Animation(obstacles, result)
    animation.construct()
    animation.save_animation("animation.gif")

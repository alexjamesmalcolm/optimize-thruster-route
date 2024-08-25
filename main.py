from typing import List, Tuple
from dataclasses import dataclass
from pulp import LpProblem, LpVariable, LpMinimize, lpSum
from pprint import pprint


@dataclass
class Obstacle:
    position: Tuple[float, float]
    radius: float


@dataclass
class Result:
    thrust_decisions: List[Tuple[float, float]]
    vehicle_positions: List[Tuple[float, float]]
    vehicle_velocities: List[Tuple[float, float]]


def do_thing(
    starting_point: Tuple[float, float],
    goal: Tuple[float, float],
    obstacles: list[Obstacle],
    max_time_segments: int = 1000,
    thrust_magnitude: float = 1,
) -> Result:
    # Declare the problem
    p = LpProblem("Pathfinding", LpMinimize)
    # Objective: Minimize the distance between the goal and the vehicle at the sum of all time segments
    distance = LpVariable.dicts("distance", range(max_time_segments), lowBound=0)
    p += lpSum(distance)

    # Thrust decision at each time segment
    x_thrust = LpVariable.dicts(
        "x_thrust", range(max_time_segments), lowBound=-1, upBound=1
    )
    y_thrust = LpVariable.dicts(
        "y_thrust", range(max_time_segments), lowBound=-1, upBound=1
    )

    # Vehicle position at each time segment
    x_position = LpVariable.dicts("x_position", range(max_time_segments))
    y_position = LpVariable.dicts("y_position", range(max_time_segments))

    # Initial conditions
    p += x_position[0] == starting_point[0]
    p += y_position[0] == starting_point[1]

    # Vehicle velocity at each time segment
    x_velocity = LpVariable.dicts("x_velocity", range(max_time_segments))
    y_velocity = LpVariable.dicts("y_velocity", range(max_time_segments))

    # Vehicle thrust effect on velocity at each time segment
    p += x_velocity[0] == 0
    p += y_velocity[0] == 0
    for i in range(max_time_segments - 1):
        p += x_velocity[i + 1] == x_velocity[i] + x_thrust[i] * thrust_magnitude
        p += y_velocity[i + 1] == y_velocity[i] + y_thrust[i] * thrust_magnitude

    # Vehicle velocity effect on position at each time segment
    p += x_velocity[0] == 0
    p += y_velocity[0] == 0
    for i in range(max_time_segments - 1):
        p += x_position[i + 1] == x_position[i] + x_velocity[i]
        p += y_position[i + 1] == y_position[i] + y_velocity[i]

    # Northern distance at each time segment
    north_distance = LpVariable.dicts(
        "north_distance", range(max_time_segments), lowBound=0
    )
    # Constraint: Northern distance is greater than or equal to the positive y position of the vehicle and the goal
    for i in range(max_time_segments):
        p += north_distance[i] >= y_position[i] - goal[1]

    # Eastern distance at each time segment
    east_distance = LpVariable.dicts(
        "east_distance", range(max_time_segments), lowBound=0
    )
    # Constraint: Eastern distance is greater than or equal to the positive x position of the vehicle and the goal
    for i in range(max_time_segments):
        p += east_distance[i] >= x_position[i] - goal[0]

    # Southern distance at each time segment
    south_distance = LpVariable.dicts(
        "south_distance", range(max_time_segments), lowBound=0
    )
    # Constraint: Southern distance is greater than or equal to the negative y position of the vehicle and the goal
    for i in range(max_time_segments):
        p += south_distance[i] >= -y_position[i] + goal[1]

    # Western distance at each time segment
    west_distance = LpVariable.dicts(
        "west_distance", range(max_time_segments), lowBound=0
    )
    # Constraint: Western distance is greater than or equal to the negative x position of the vehicle and the goal
    for i in range(max_time_segments):
        p += west_distance[i] >= -x_position[i] + goal[0]

    # Constraint: The distance between the vehicle and the goal is the sum of the northern, eastern, southern, and western distances
    for i in range(max_time_segments):
        p += (
            distance[i]
            >= north_distance[i]
            + east_distance[i]
            + south_distance[i]
            + west_distance[i]
        )

    p.solve()

    return Result(
        thrust_decisions=[
            (x_thrust[i].value(), y_thrust[i].value()) for i in range(max_time_segments)
        ],
        vehicle_positions=[
            (x_position[i].value(), y_position[i].value())
            for i in range(max_time_segments)
        ],
        vehicle_velocities=[
            (x_velocity[i].value(), y_velocity[i].value())
            for i in range(max_time_segments)
        ],
    )


if __name__ == "__main__":
    result = do_thing(
        starting_point=(0, 0.5),
        goal=(1, 1),
        obstacles=[],
        max_time_segments=20,
        thrust_magnitude=0.05,
    )
    pprint(result)

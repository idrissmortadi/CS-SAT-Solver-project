import random
import time
from concurrent.futures import ProcessPoolExecutor, as_completed

import matplotlib.pyplot as plt

from src.utils import (
    Position,
    Robot,
    animate_solution,
    solve_warehouse_problem,
    visualize_solution_grid,
)


def randomize_warehouse():
    """
    Solve the warehouse problem with random obstacles.
    """
    random.seed(42)
    # Define the warehouse dimensions
    width, height = 16, 16

    # Define robots
    num_robots = random.randint(12, 12)
    robots = []
    used_positions = set()
    for i in range(1, num_robots + 1):
        # Generate unique start and goal positions for each robot
        while True:
            start = Position(
                random.randint(0, width - 1), random.randint(0, height - 1)
            )
            goal = Position(random.randint(0, width - 1), random.randint(0, height - 1))
            if (
                start != goal
                and start not in used_positions
                and goal not in used_positions
            ):
                used_positions.add(start)
                used_positions.add(goal)
                robots.append(Robot(i, start, goal))
                break

    # Define obstacles
    obstacles = set()
    num_obstacles = random.randint(16, 16)  # random number of obstacles
    # Collect forbidden positions: initial and goal positions of all robots
    forbidden = {robot.start for robot in robots} | {robot.goal for robot in robots}
    while len(obstacles) < num_obstacles:
        pos = Position(random.randint(0, width - 1), random.randint(0, height - 1))
        if pos not in forbidden:
            obstacles.add(pos)

    # Time horizon
    time_horizon = 32

    # Solve the problem
    paths = solve_warehouse_problem(width, height, robots, obstacles, time_horizon)

    if paths:
        visualize_solution_grid(
            width,
            height,
            robots,
            obstacles,
            paths,
            figure_filename="report/figures/randomized_warehouse.png",
        )
        animate_solution(
            width,
            height,
            robots,
            obstacles,
            paths,
            filename="report/figures/randomized_warehouse.gif",
        )
    else:
        print("No solution found.")


def basic_warehouse():
    """
    Solve the basic warehouse problem with 3 robots and 8x8 grid.
    """
    # Define the warehouse dimensions
    width, height = 8, 8

    # Define robots
    robots = [
        Robot(1, Position(0, 0), Position(7, 7)),
        Robot(2, Position(7, 0), Position(0, 7)),
        Robot(3, Position(0, 7), Position(7, 0)),
    ]

    # Define obstacles
    obstacles = {
        Position(4, 0),
        Position(4, 1),
        # Position(4, 2),
        Position(4, 3),
        Position(4, 4),
        Position(4, 5),
        Position(4, 6),
        Position(4, 7),
    }

    # Time horizon
    time_horizon = 16

    # Solve the problem
    paths = solve_warehouse_problem(width, height, robots, obstacles, time_horizon)

    if paths:
        visualize_solution_grid(
            width,
            height,
            robots,
            obstacles,
            paths,
            figure_filename="report/figures/basic_warehouse.png",
        )
        animate_solution(width, height, robots, obstacles, paths)
    else:
        print("No solution found.")


def worker_simulation(width, height, num, time_horizon):
    """
    Run a single simulation of the warehouse problem and return
    a tuple with key (width, num) and the elapsed solver time.
    """
    # Generate robots with unique start/goal positions.
    robots = []
    used_positions = set()
    for i in range(1, num + 1):
        while True:
            start = Position(
                random.randint(0, width - 1), random.randint(0, height - 1)
            )
            goal = Position(random.randint(0, width - 1), random.randint(0, height - 1))
            if (
                start != goal
                and start not in used_positions
                and goal not in used_positions
            ):
                used_positions.add(start)
                used_positions.add(goal)
                robots.append(Robot(i, start, goal))
                break

    # Define obstacles randomly, avoiding robot start/goal positions.
    obstacles = set()
    num_obstacles = random.randint(3, 8)
    forbidden = {robot.start for robot in robots} | {robot.goal for robot in robots}
    while len(obstacles) < num_obstacles:
        pos = Position(random.randint(0, width - 1), random.randint(0, height - 1))
        if pos not in forbidden:
            obstacles.add(pos)

    # Measure solver execution time.
    start_time = time.perf_counter()
    paths = solve_warehouse_problem(width, height, robots, obstacles, time_horizon)
    elapsed = time.perf_counter() - start_time
    print(
        f"Warehouse: {width}x{height}, Robots: {num}, Time: {elapsed:.4f} sec, Success: {paths is not None}"
    )
    return ((width, num), elapsed)


def test_complexity():
    """
    Evaluate the complexity of the warehouse problem.

    Iterate over different warehouse sizes and numbers of robots
    to evaluate the performance of the solver, and then generate a plot.
    """
    sizes = [(8, 8), (10, 10), (12, 12), (14, 14)]
    robot_counts = [3, 5, 7, 10, 12, 15]
    time_horizon = 32

    results = {}  # key: (warehouse_width, num_robots), value: execution time

    # Parallelize simulation using ProcessPoolExecutor.
    with ProcessPoolExecutor(max_workers=4) as executor:
        futures = []
        for width, height in sizes:
            for num in robot_counts:
                futures.append(
                    executor.submit(worker_simulation, width, height, num, time_horizon)
                )
        for future in as_completed(futures):
            key, elapsed = future.result()
            results[key] = elapsed

    # Plot the results: one curve per warehouse size.
    plt.figure()
    for width, height in sizes:
        times = [results[(width, num)] for num in robot_counts]
        plt.plot(robot_counts, times, marker="o", label=f"{width}x{height}")
    plt.xlabel("Number of Robots")
    plt.ylabel("Solver Time (seconds)")
    plt.title("Warehouse Problem Complexity")
    plt.legend()
    plt.grid(True)
    plt.savefig("report/figures/complexity_plot.png")


if __name__ == "__main__":
    # basic_warehouse()
    randomize_warehouse()
    # test_complexity()

from typing import Dict, List, Optional, Set
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import seaborn as sns
from matplotlib import animation
from warehouse_path_planner import Position, Robot, WarehousePathPlanner


def solve_warehouse_problem(
    width: int,
    height: int,
    robots: List[Robot],
    obstacles: Set[Position],
    time_horizon: int,
) -> Optional[Dict[int, List[Position]]]:
    """
    Solve the warehouse path planning problem.

    Args:
        width: Width of the warehouse grid
        height: Height of the warehouse grid
        robots: List of robots with their start and goal positions
        obstacles: Set of obstacle positions
        time_horizon: Maximum number of time steps allowed

    Returns:
        Dictionary mapping robot IDs to their paths if a solution exists, None otherwise
    """
    # Input validation
    if width <= 0 or height <= 0:
        raise ValueError("Width and height must be positive")
    if time_horizon < 0:
        raise ValueError("Time horizon must be non-negative")
    if not robots:
        raise ValueError("At least one robot must be specified")

    # Validate robot positions
    for robot in robots:
        if not (0 <= robot.start.x < width and 0 <= robot.start.y < height):
            raise ValueError(
                f"Robot {robot.id} start position is outside the warehouse bounds"
            )
        if not (0 <= robot.goal.x < width and 0 <= robot.goal.y < height):
            raise ValueError(
                f"Robot {robot.id} goal position is outside the warehouse bounds"
            )
        if Position(robot.start.x, robot.start.y) in obstacles:
            raise ValueError(
                f"Robot {robot.id} start position overlaps with an obstacle"
            )
        if Position(robot.goal.x, robot.goal.y) in obstacles:
            raise ValueError(
                f"Robot {robot.id} goal position overlaps with an obstacle"
            )

    planner = WarehousePathPlanner(width, height, time_horizon)

    # Add all constraints
    planner.add_initial_positions(robots)
    planner.add_goal_positions(robots)
    planner.add_obstacle_constraints(obstacles, robots)
    planner.add_movement_constraints(robots)
    planner.add_collision_avoidance(robots)
    planner.add_position_switching_prohibition(robots)

    # Solve the problem
    solution = planner.solve()
    if solution is None:
        return None

    # Decode and return the solution
    return planner.decode_solution(solution)


def test_simple_case():
    """Test a very simple case to verify basic functionality."""
    print("\nTesting simple case...")

    # Simple 2x2 grid with one robot
    width, height = 2, 2
    robots = [Robot(1, Position(0, 0), Position(1, 1))]
    obstacles = set()  # No obstacles

    # Try with different time horizons
    for time_horizon in range(1, 4):
        print(f"\nTrying simple case with time horizon = {time_horizon}")
        try:
            paths = solve_warehouse_problem(
                width, height, robots, obstacles, time_horizon
            )

            if paths:
                print("Solution found!")
                for robot_id, path in sorted(paths.items()):
                    print(f"Robot {robot_id} path:")
                    for t, pos in enumerate(path):
                        move_type = (
                            "stay" if (t > 0 and path[t] == path[t - 1]) else "move"
                        )
                        print(f"Time {t}: ({pos.x}, {pos.y}) - {move_type}")
            else:
                print(f"No solution found for time horizon {time_horizon}")
        except Exception as e:
            print(f"Error: {e}")


def animate_solution(
    width: int,
    height: int,
    robots: List[Robot],
    obstacles: Set[Position],
    paths: Dict[int, List[Position]],
    filename="robot_animation.gif",
):
    """Animates all robots moving along their paths and saves as GIF."""
    sns.set_theme(style="whitegrid")
    fig, ax = plt.subplots(figsize=(10, 10))

    max_time = max(len(path) for path in paths.values())
    colors = sns.color_palette("husl", len(robots))

    def update(t):
        ax.clear()

        # Draw grid and obstacles
        for x in range(width):
            for y in range(height):
                ax.add_patch(
                    patches.Rectangle(
                        (x, y), 1, 1, fill=False, edgecolor="gray", lw=0.5
                    )
                )

        for obs in obstacles:
            ax.add_patch(patches.Rectangle((obs.x, obs.y), 1, 1, color="black"))

        # Draw paths and robots up to current time t
        for i, robot in enumerate(robots):
            path = paths[robot.id]
            current_t = min(t, len(path) - 1)
            robot_color = colors[i]

            # Draw path history and timesteps
            for j in range(current_t):
                pos = path[j]
                next_pos = path[j + 1]
                ax.arrow(
                    pos.x + 0.5,
                    pos.y + 0.5,
                    next_pos.x - pos.x,
                    next_pos.y - pos.y,
                    head_width=0.2,
                    head_length=0.2,
                    fc=robot_color,
                    ec=robot_color,
                    alpha=0.3,
                )
                ax.text(pos.x + 0.1, pos.y + 0.1, str(j), color=robot_color, fontsize=8)

            # Draw current robot position
            current_pos = path[current_t]
            ax.add_patch(
                patches.Circle(
                    (current_pos.x + 0.5, current_pos.y + 0.5), 0.3, color=robot_color
                )
            )
            ax.text(
                current_pos.x + 0.5,
                current_pos.y + 0.5,
                str(robot.id),
                ha="center",
                va="center",
                color="white",
            )

            # Mark start and goal
            ax.add_patch(
                patches.Circle(
                    (robot.start.x + 0.5, robot.start.y + 0.5),
                    0.2,
                    color=robot_color,
                    alpha=0.5,
                )
            )
            ax.add_patch(
                patches.Circle(
                    (robot.goal.x + 0.5, robot.goal.y + 0.5),
                    0.2,
                    color=robot_color,
                    alpha=0.5,
                )
            )

        ax.set_xlim(0, width)
        ax.set_ylim(0, height)
        ax.set_aspect("equal")
        ax.set_title(f"Time step: {t}")

    anim = animation.FuncAnimation(
        fig,
        update,  # type: ignore
        frames=max_time + 1,
        interval=500,
        repeat=False,
    )
    anim.save(filename, writer="pillow")
    plt.close()


def visualize_solution_grid(
    width: int,
    height: int,
    robots: List[Robot],
    obstacles: Set[Position],
    paths: Dict[int, List[Position]],
):
    """Visualizes the solution in a grid of plots, one per robot."""
    sns.set_theme(style="whitegrid", font_scale=1.2)

    num_robots = len(robots)
    cols = 3  # Set the number of columns in the grid
    rows = (num_robots + cols - 1) // cols  # Calculate rows to fit all robots

    fig, axes = plt.subplots(rows, cols, figsize=(6 * cols, 6 * rows))
    axes = axes.flatten()  # Flatten for easy iteration

    colors = sns.color_palette("husl", num_robots)  # Assign unique colors for robots

    for i, robot in enumerate(robots):
        ax = axes[i]
        robot_id = robot.id
        robot_color = colors[i]
        path = paths[robot_id]

        # Draw grid
        for x in range(width):
            for y in range(height):
                ax.add_patch(
                    patches.Rectangle(
                        (x, y), 1, 1, fill=False, edgecolor="gray", lw=0.5
                    )
                )

        # Draw obstacles
        for obs in obstacles:
            ax.add_patch(patches.Rectangle((obs.x, obs.y), 1, 1, color="black"))

        # Draw robot's path
        for t, pos in enumerate(path):
            ax.text(
                pos.x + 0.1,
                pos.y + 0.1,
                f"{t}",
                color=robot_color,
                fontsize=9,
                weight="bold",
            )
            if t > 0:
                prev_pos = path[t - 1]
                dx = pos.x - prev_pos.x
                dy = pos.y - prev_pos.y
                ax.arrow(
                    prev_pos.x + 0.5,
                    prev_pos.y + 0.5,
                    dx,
                    dy,
                    head_width=0.2,
                    head_length=0.2,
                    fc=robot_color,
                    ec=robot_color,
                )

        # Robot start and goal
        ax.add_patch(
            patches.Circle(
                (robot.start.x + 0.5, robot.start.y + 0.5),
                0.3,
                color=robot_color,
                alpha=0.5,
                label="Start",
            )
        )
        ax.text(
            robot.start.x + 0.5,
            robot.start.y + 0.5,
            "S",
            color="black",
            fontsize=9,
            weight="bold",
            ha="center",  # Horizontal alignment
            va="center",  # Vertical alignment
        )
        ax.add_patch(
            patches.Circle(
                (robot.goal.x + 0.5, robot.goal.y + 0.5),
                0.3,
                color=robot_color,
                edgecolor="black",
                linewidth=2,
                label="Goal",
            )
        )
        ax.text(
            robot.goal.x + 0.5,
            robot.goal.y + 0.5,
            "G",
            color="black",
            fontsize=9,
            weight="bold",
            ha="center",  # Horizontal alignment
            va="center",  # Vertical alignment
        )

        # Title and axis settings
        ax.set_xlim(0, width)
        ax.set_ylim(0, height)
        ax.set_aspect("equal")
        ax.set_xticks(range(width))
        ax.set_yticks(range(height))
        ax.set_title(f"Robot {robot_id} Path", fontsize=14)
        # ax.legend(loc="upper left")

    # Hide any unused axes
    for j in range(num_robots, len(axes)):
        axes[j].axis("off")

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    # Define the warehouse dimensions
    width, height = 8, 8

    # Define robots
    robots = [
        Robot(1, Position(0, 0), Position(7, 7)),
        Robot(2, Position(7, 0), Position(0, 7)),
        Robot(3, Position(0, 7), Position(7, 0)),
        # Robot(4, Position(7, 7), Position(0, 0)),
        # Robot(5, Position(3, 3), Position(5, 5)),
        # Robot(6, Position(5, 5), Position(3, 3)),
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
        visualize_solution_grid(width, height, robots, obstacles, paths)
        animate_solution(width, height, robots, obstacles, paths)
    else:
        print("No solution found.")

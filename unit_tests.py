
# Run tests by running "pytest unit_tests.py -v" in terminal

import pytest
from typing import Set, List, Dict
from main import Position, Robot, solve_warehouse_problem

def test_basic_movement():
    """Test simple movement from start to goal without obstacles."""
    width, height = 2, 2
    robots = [Robot(1, Position(0, 0), Position(1, 1))]
    obstacles: Set[Position] = set()
    time_horizon = 2

    paths = solve_warehouse_problem(width, height, robots, obstacles, time_horizon)
    
    assert paths is not None
    assert len(paths[1]) == time_horizon + 1  # Path includes start and end positions
    assert paths[1][0] == Position(0, 0)  # Start position
    assert paths[1][-1] == Position(1, 1)  # Goal position

def test_obstacle_avoidance():
    """Test that robots avoid obstacles."""
    width, height = 3, 3
    robots = [Robot(1, Position(0, 0), Position(2, 2))]
    obstacles = {Position(1, 1)}  # Obstacle in the center
    time_horizon = 4

    paths = solve_warehouse_problem(width, height, robots, obstacles, time_horizon)
    
    assert paths is not None
    # Verify that the robot never occupies the obstacle position
    for positions in paths.values():
        assert Position(1, 1) not in positions

def test_collision_avoidance_impossible():
    """Test that the planner correctly identifies impossible passing scenarios in narrow corridors."""
    width, height = 3, 1  # 1-high corridor makes passing impossible
    robots = [
        Robot(1, Position(0, 0), Position(2, 0)),
        Robot(2, Position(2, 0), Position(0, 0))
    ]
    obstacles: Set[Position] = set()
    time_horizon = 4

    paths = solve_warehouse_problem(width, height, robots, obstacles, time_horizon)
    # Should return None as it's physically impossible for robots to pass in a 1-high corridor
    assert paths is None

def test_path_continuity():
    """Test that robots move only to adjacent cells."""
    width, height = 4, 4
    robots = [Robot(1, Position(0, 0), Position(3, 3))]
    obstacles: Set[Position] = set()
    time_horizon = 6

    paths = solve_warehouse_problem(width, height, robots, obstacles, time_horizon)
    
    assert paths is not None
    # Check that each move is to an adjacent cell
    for robot_path in paths.values():
        for t in range(1, len(robot_path)):
            prev_pos = robot_path[t-1]
            curr_pos = robot_path[t]
            manhattan_dist = abs(prev_pos.x - curr_pos.x) + abs(prev_pos.y - curr_pos.y)
            assert manhattan_dist <= 1  # Can only move to adjacent cells or stay put

def test_invalid_dimensions():
    """Test that invalid warehouse dimensions raise appropriate errors."""
    with pytest.raises(ValueError):
        solve_warehouse_problem(0, 5, [Robot(1, Position(0, 0), Position(1, 1))], set(), 5)
    
    with pytest.raises(ValueError):
        solve_warehouse_problem(5, -1, [Robot(1, Position(0, 0), Position(1, 1))], set(), 5)

def test_invalid_robot_positions():
    """Test that invalid robot positions raise appropriate errors."""
    width, height = 3, 3
    
    # Test start position outside bounds
    with pytest.raises(ValueError):
        robots = [Robot(1, Position(3, 3), Position(1, 1))]
        solve_warehouse_problem(width, height, robots, set(), 5)
    
    # Test goal position outside bounds
    with pytest.raises(ValueError):
        robots = [Robot(1, Position(1, 1), Position(3, 3))]
        solve_warehouse_problem(width, height, robots, set(), 5)

def test_robot_on_obstacle():
    """Test that placing robots on obstacles raises appropriate errors."""
    width, height = 3, 3
    obstacles = {Position(1, 1)}
    
    # Test start position on obstacle
    with pytest.raises(ValueError):
        robots = [Robot(1, Position(1, 1), Position(2, 2))]
        solve_warehouse_problem(width, height, robots, obstacles, 5)
    
    # Test goal position on obstacle
    with pytest.raises(ValueError):
        robots = [Robot(1, Position(0, 0), Position(1, 1))]
        solve_warehouse_problem(width, height, robots, obstacles, 5)

def test_no_solution():
    """Test case where no solution exists."""
    width, height = 3, 3
    robots = [Robot(1, Position(0, 0), Position(2, 2))]
    # Create a wall of obstacles that blocks the path
    obstacles = {Position(1, 0), Position(1, 1), Position(1, 2)}
    time_horizon = 5

    paths = solve_warehouse_problem(width, height, robots, obstacles, time_horizon)
    assert paths is None

def test_position_switching_impossible():
    """Test that the planner correctly identifies impossible position switching in narrow corridors."""
    width, height = 2, 1  # 1-high corridor makes switching impossible
    robots = [
        Robot(1, Position(0, 0), Position(1, 0)),
        Robot(2, Position(1, 0), Position(0, 0))
    ]
    obstacles: Set[Position] = set()
    time_horizon = 3

    paths = solve_warehouse_problem(width, height, robots, obstacles, time_horizon)
    # Should return None as it's physically impossible for robots to switch positions in a 1-high corridor
    assert paths is None

def test_passing_possible():
    """Test that robots can pass each other when there is enough space."""
    width, height = 2, 2  # 2-high corridor makes passing possible
    robots = [
        Robot(1, Position(0, 0), Position(1, 0)),
        Robot(2, Position(1, 0), Position(0, 0))
    ]
    obstacles: Set[Position] = set()
    time_horizon = 4

    paths = solve_warehouse_problem(width, height, robots, obstacles, time_horizon)
    assert paths is not None
    # Verify both robots reach their goals
    assert paths[1][-1] == Position(1, 0)
    assert paths[2][-1] == Position(0, 0)

if __name__ == "__main__":
    pytest.main([__file__])
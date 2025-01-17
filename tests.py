import unittest
from main_ import Position, Robot, solve_warehouse_problem

class TestWarehousePlanner(unittest.TestCase):
    def test_single_robot_no_obstacles(self):
        """Test a single robot moving in an empty 2x2 grid."""
        width, height = 2, 2
        robots = [Robot(1, Position(0, 0), Position(1, 1))]
        obstacles = set()
        time_horizon = 2

        paths = solve_warehouse_problem(width, height, robots, obstacles, time_horizon)
        
        self.assertIsNotNone(paths, "Should find a solution")
        self.assertEqual(len(paths), 1, "Should have path for one robot")
        self.assertEqual(len(paths[1]), time_horizon + 1, "Path should have correct length")
        self.assertEqual(paths[1][0], Position(0, 0), "Should start at start position")
        self.assertEqual(paths[1][-1], Position(1, 1), "Should end at goal position")

    def test_single_robot_with_obstacle(self):
        """Test a single robot navigating around an obstacle."""
        width, height = 3, 3
        robots = [Robot(1, Position(0, 0), Position(2, 2))]
        obstacles = {Position(1, 1)}  # Obstacle in center
        time_horizon = 4

        paths = solve_warehouse_problem(width, height, robots, obstacles, time_horizon)
        
        self.assertIsNotNone(paths, "Should find a solution")
        path = paths[1]
        self.assertNotIn(Position(1, 1), path, "Path should not include obstacle position")

    def test_two_robots_collision_avoidance(self):
        """Test two robots avoiding collision in a simple scenario."""
        width, height = 2, 1  # A 2x1 corridor
        robots = [
            Robot(1, Position(0, 0), Position(1, 0)),
            Robot(2, Position(1, 0), Position(0, 0))
        ]
        obstacles = set()
        time_horizon = 3

        paths = solve_warehouse_problem(width, height, robots, obstacles, time_horizon)
        
        self.assertIsNone(paths, "Should not find a solution (impossible without collision)")

    def test_invalid_inputs(self):
        """Test that invalid inputs raise appropriate exceptions."""
        with self.assertRaises(ValueError):
            # Test negative width
            solve_warehouse_problem(-1, 5, [Robot(1, Position(0, 0), Position(1, 1))], set(), 5)
        
        with self.assertRaises(ValueError):
            # Test robot starting at obstacle
            solve_warehouse_problem(3, 3, 
                [Robot(1, Position(1, 1), Position(2, 2))], 
                {Position(1, 1)}, 
                5)
        
        with self.assertRaises(ValueError):
            # Test robot goal at obstacle
            solve_warehouse_problem(3, 3, 
                [Robot(1, Position(0, 0), Position(1, 1))], 
                {Position(1, 1)}, 
                5)

    def test_simple_path_verification(self):
        """Test that paths are continuous (no teleporting)."""
        width, height = 2, 2
        robots = [Robot(1, Position(0, 0), Position(1, 1))]
        obstacles = set()
        time_horizon = 2

        paths = solve_warehouse_problem(width, height, robots, obstacles, time_horizon)
        
        self.assertIsNotNone(paths, "Should find a solution")
        path = paths[1]
        
        # Verify each move is to an adjacent cell (including diagonals)
        for i in range(len(path) - 1):
            dx = abs(path[i+1].x - path[i].x)
            dy = abs(path[i+1].y - path[i].y)
            self.assertTrue(dx <= 1 and dy <= 1, 
                          f"Invalid move from {path[i]} to {path[i+1]}")

    def test_minimum_time_horizon(self):
        """Test that the solution respects the minimum possible time horizon."""
        width, height = 2, 2
        robots = [Robot(1, Position(0, 0), Position(1, 1))]
        obstacles = set()
        
        # Test with time horizon that's too short
        too_short = 1
        paths = solve_warehouse_problem(width, height, robots, obstacles, too_short)
        self.assertIsNone(paths, "Should not find solution with too short time horizon")
        
        # Test with minimum required time
        minimum = 2
        paths = solve_warehouse_problem(width, height, robots, obstacles, minimum)
        self.assertIsNotNone(paths, "Should find solution with minimum required time")

def main():
    # Run the tests
    unittest.main(verbosity=2)

if __name__ == '__main__':
    main()
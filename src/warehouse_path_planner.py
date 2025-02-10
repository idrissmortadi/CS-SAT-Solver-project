import itertools
from dataclasses import dataclass
from typing import Dict, List, Optional, Set

from pysat.formula import CNF
from pysat.solvers import Glucose3


@dataclass(frozen=True)  # Making Position immutable and hashable
class Position:
    x: int
    y: int

    def _hash_(self):
        return hash((self.x, self.y))

    def _eq_(self, other):
        if not isinstance(other, Position):
            return NotImplemented
        return self.x == other.x and self.y == other.y


@dataclass
class Robot:
    id: int
    start: Position
    goal: Position


class WarehousePathPlanner:
    def __init__(self, width: int, height: int, time_horizon: int):
        self.width = width
        self.height = height
        self.time_horizon = time_horizon
        self.cnf = CNF()
        self.var_map = {}
        self.next_var = 1
        # Valid movements: stay, up, right, down, left
        self.moves = [(0, 0), (0, 1), (1, 0), (0, -1), (-1, 0)]

    def create_variable(self, robot_id: int, x: int, y: int, t: int) -> int:
        """Create a variable representing robot's position at time t."""
        key = f"R{robot_id}X{x}Y{y}T{t}"  # Removed underscore separator
        if key not in self.var_map:
            self.var_map[key] = self.next_var
            self.next_var += 1
        return self.var_map[key]

    def add_initial_positions(self, robots: List[Robot]):
        """Add constraints for initial positions of all robots."""
        for robot in robots:
            # Add positive clause for start position
            start_var = self.create_variable(robot.id, robot.start.x, robot.start.y, 0)
            self.cnf.append([start_var])

            # Add "at most one position" constraints
            for x in range(self.width):
                for y in range(self.height):
                    var = self.create_variable(robot.id, x, y, 0)
                    if x != robot.start.x or y != robot.start.y:
                        self.cnf.append([-var])  # Cannot be at any other position

    def add_goal_positions(self, robots: List[Robot]):
        """Add constraints for goal positions of all robots."""
        for robot in robots:
            # Add positive clause for goal position
            goal_var = self.create_variable(
                robot.id, robot.goal.x, robot.goal.y, self.time_horizon
            )
            self.cnf.append([goal_var])

            # Add "at most one position" constraints for final time step
            for x in range(self.width):
                for y in range(self.height):
                    if x != robot.goal.x or y != robot.goal.y:
                        var = self.create_variable(robot.id, x, y, self.time_horizon)
                        self.cnf.append([-var])

    def add_obstacle_constraints(self, obstacles: Set[Position], robots: List[Robot]):
        """Add constraints preventing robots from occupying obstacle positions."""
        for obstacle in obstacles:
            if not (0 <= obstacle.x < self.width and 0 <= obstacle.y < self.height):
                raise ValueError(
                    f"Obstacle at position ({obstacle.x}, {obstacle.y}) is outside the warehouse bounds"
                )

            for robot in robots:
                for t in range(self.time_horizon + 1):
                    var = self.create_variable(robot.id, obstacle.x, obstacle.y, t)
                    self.cnf.append([-var])

    def add_movement_constraints(self, robots: List[Robot]):
        """Add constraints for valid movements between time steps."""
        for robot in robots:
            for t in range(self.time_horizon + 1):
                # At each time step, robot must be at exactly one position
                positions_at_t = []
                for x in range(self.width):
                    for y in range(self.height):
                        var = self.create_variable(robot.id, x, y, t)
                        positions_at_t.append(var)

                # At least one position
                self.cnf.append(positions_at_t)

                # At most one position
                for pos1, pos2 in itertools.combinations(positions_at_t, 2):
                    self.cnf.append([-pos1, -pos2])

                # If not the last time step, add movement constraints
                if t < self.time_horizon:
                    for x in range(self.width):
                        for y in range(self.height):
                            current_pos = self.create_variable(robot.id, x, y, t)

                            # Generate valid next positions (including staying in place)
                            next_positions = []
                            for (
                                dx,
                                dy,
                            ) in self.moves:  # includes (0,0) for staying in place
                                next_x, next_y = x + dx, y + dy
                                if (
                                    0 <= next_x < self.width
                                    and 0 <= next_y < self.height
                                ):
                                    next_pos = self.create_variable(
                                        robot.id, next_x, next_y, t + 1
                                    )
                                    next_positions.append(next_pos)

                            # If at current position, must move to one of valid next positions
                            if next_positions:
                                self.add_implication(current_pos, next_positions)

    def add_collision_avoidance(self, robots: List[Robot]):
        """Add constraints preventing robots from occupying the same position or crossing paths."""
        # Vertex collision avoidance
        for t in range(self.time_horizon + 1):
            for x in range(self.width):
                for y in range(self.height):
                    # No two robots can be at the same position at the same time
                    robot_vars = []
                    for robot in robots:
                        var = self.create_variable(robot.id, x, y, t)
                        robot_vars.append(var)

                    # At most one robot can be at any position
                    for var1, var2 in itertools.combinations(robot_vars, 2):
                        self.cnf.append([-var1, -var2])

    def add_position_switching_prohibition(self, robots: List[Robot]):
        """Add constraints preventing robots from switching positions."""
        for t in range(self.time_horizon):
            for x1 in range(self.width):
                for y1 in range(self.height):
                    for dx, dy in self.moves[1:]:  # Exclude staying in place
                        x2, y2 = x1 + dx, y1 + dy
                        if 0 <= x2 < self.width and 0 <= y2 < self.height:
                            for r1, r2 in itertools.combinations(robots, 2):
                                # Variables for positions at time t
                                r1_pos1_t = self.create_variable(r1.id, x1, y1, t)
                                r2_pos2_t = self.create_variable(r2.id, x2, y2, t)

                                # Variables for positions at time t+1
                                r1_pos2_t1 = self.create_variable(r1.id, x2, y2, t + 1)
                                r2_pos1_t1 = self.create_variable(r2.id, x1, y1, t + 1)

                                # Prevent position switching
                                self.cnf.append(
                                    [-r1_pos1_t, -r2_pos2_t, -r1_pos2_t1, -r2_pos1_t1]
                                )

    def add_implication(self, antecedent: int, consequents: List[int]):
        """Add A → (B₁ ∨ B₂ ∨ ... ∨ Bₙ) constraint."""
        self.cnf.append([-antecedent] + consequents)

    def solve(self) -> Optional[Dict[str, bool]]:
        """Solve the SAT problem and return the solution if one exists."""
        with Glucose3() as solver:
            solver.append_formula(self.cnf.clauses)

            if solver.solve():
                model = solver.get_model()
                if not isinstance(model, list):
                    raise ValueError("No model found")

                return {
                    var_name: lit > 0
                    for var_name, var_num in self.var_map.items()
                    for lit in model
                    if abs(lit) == var_num
                }

            return None

    def decode_solution(self, solution: Dict[str, bool]) -> Dict[int, List[Position]]:
        """Convert SAT solution to robot paths."""
        paths = {}
        for var_name, is_true in solution.items():
            if is_true:
                # Parse variable name (format: R{id}X{x}Y{y}T{t})
                # Remove the letters and split the remaining string
                numbers = "".join(c if c.isdigit() else " " for c in var_name).split()
                robot_id, x, y, t = map(int, numbers)

                if robot_id not in paths:
                    paths[robot_id] = [None] * (self.time_horizon + 1)
                paths[robot_id][t] = Position(x, y)

        # Verify paths are complete
        for path in paths.values():
            if None in path:
                raise ValueError("Invalid solution: incomplete path detected")

        return paths

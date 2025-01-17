from pysat.formula import CNF
from pysat.solvers import Glucose3
from typing import List, Tuple, Set, Dict, Optional
from dataclasses import dataclass
import itertools

@dataclass(frozen=True)  # Making Position immutable and hashable
class Position:
    x: int
    y: int
    
    def __hash__(self):
        return hash((self.x, self.y))
    
    def __eq__(self, other):
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
        key = f"R{robot_id}_X{x}Y{y}T{t}"
        if key not in self.var_map:
            self.var_map[key] = self.next_var
            self.next_var += 1
        return self.var_map[key]

    def add_initial_positions(self, robots: List[Robot]):
        """Add constraints for initial positions of all robots."""
        for robot in robots:
            # Robot must be at its start position at time 0
            var = self.create_variable(robot.id, robot.start.x, robot.start.y, 0)
            self.cnf.append([var])
            
            # Robot cannot be anywhere else at time 0
            for x in range(self.width):
                for y in range(self.height):
                    if x != robot.start.x or y != robot.start.y:
                        var = self.create_variable(robot.id, x, y, 0)
                        self.cnf.append([-var])

    def add_goal_positions(self, robots: List[Robot]):
        """Add constraints for goal positions of all robots."""
        for robot in robots:
            # Robot must be at its goal position at final time
            var = self.create_variable(robot.id, robot.goal.x, robot.goal.y, self.time_horizon)
            self.cnf.append([var])

    def add_obstacle_constraints(self, obstacles: Set[Position], robots: List[Robot]):
        """Add constraints preventing robots from occupying obstacle positions."""
        for obstacle in obstacles:
            for robot in robots:
                for t in range(self.time_horizon + 1):
                    var = self.create_variable(robot.id, obstacle.x, obstacle.y, t)
                    self.cnf.append([-var])

    def add_movement_constraints(self, robots: List[Robot]):
        """Add constraints for valid movements between time steps."""
        for robot in robots:
            for t in range(self.time_horizon):
                for x in range(self.width):
                    for y in range(self.height):
                        current_pos = self.create_variable(robot.id, x, y, t)
                        
                        # If robot is at (x,y) at time t, it must move to an adjacent cell at t+1
                        next_positions = []
                        for dx, dy in self.moves:
                            next_x = x + dx
                            next_y = y + dy
                            if 0 <= next_x < self.width and 0 <= next_y < self.height:
                                next_pos = self.create_variable(robot.id, next_x, next_y, t + 1)
                                next_positions.append(next_pos)
                        
                        # Robot must be in exactly one of the valid next positions
                        if next_positions:
                            # If at current position, must move to one of valid next positions
                            self.add_implication(current_pos, next_positions)
                            # Cannot be in multiple positions at once
                            for pos1, pos2 in itertools.combinations(next_positions, 2):
                                self.cnf.append([-pos1, -pos2])

    def add_collision_avoidance(self, robots: List[Robot]):
        """Add constraints preventing robots from occupying the same position."""
        for t in range(self.time_horizon + 1):
            for x in range(self.width):
                for y in range(self.height):
                    # No two robots can be at the same position at the same time
                    for r1, r2 in itertools.combinations(robots, 2):
                        var1 = self.create_variable(r1.id, x, y, t)
                        var2 = self.create_variable(r2.id, x, y, t)
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
                                # If r1 at (x1,y1) and r2 at (x2,y2) at time t
                                r1_pos1 = self.create_variable(r1.id, x1, y1, t)
                                r2_pos1 = self.create_variable(r2.id, x2, y2, t)
                                # Then they cannot switch positions at t+1
                                r1_pos2 = self.create_variable(r1.id, x2, y2, t + 1)
                                r2_pos2 = self.create_variable(r2.id, x1, y1, t + 1)
                                
                                # (A ∧ B) → ¬(C ∧ D) becomes ¬A ∨ ¬B ∨ ¬C ∨ ¬D
                                self.cnf.append([-r1_pos1, -r2_pos1, -r1_pos2, -r2_pos2])

    def add_implication(self, antecedent: int, consequents: List[int]):
        """Add A → (B₁ ∨ B₂ ∨ ... ∨ Bₙ) constraint."""
        self.cnf.append([-antecedent] + consequents)

    def solve(self) -> Optional[Dict[str, bool]]:
        """Solve the SAT problem and return the solution if one exists."""
        solver = Glucose3()
        solver.append_formula(self.cnf.clauses)
        
        if solver.solve():
            model = solver.get_model()
            solution = {
                var_name: lit > 0
                for var_name, var_num in self.var_map.items()
                for lit in model
                if abs(lit) == var_num
            }
            solver.delete()
            return solution
        
        solver.delete()
        return None

    def decode_solution(self, solution: Dict[str, bool]) -> Dict[int, List[Position]]:
        """Convert SAT solution to robot paths."""
        paths = {}
        for var_name, is_true in solution.items():
            if is_true:
                # Parse variable name (format: R{id}_X{x}Y{y}T{t})
                parts = var_name.replace('R', '').replace('X', ' ').replace('Y', ' ').replace('T', ' ').split()
                robot_id = int(parts[0])
                x, y, t = map(int, parts[1:])
                
                if robot_id not in paths:
                    paths[robot_id] = [None] * (self.time_horizon + 1)
                paths[robot_id][t] = Position(x, y)
        
        return paths

def solve_warehouse_problem(
    width: int,
    height: int,
    robots: List[Robot],
    obstacles: Set[Position],
    time_horizon: int
) -> Optional[Dict[int, List[Position]]]:
    """
    Solve the warehouse path planning problem.
    
    Returns a dictionary mapping robot IDs to their paths,
    where each path is a list of positions at each time step.
    """
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
        
    # Decode the solution into paths
    return planner.decode_solution(solution)

# Example usage
def main():
    # Define a simple warehouse scenario
    width, height = 4, 4
    time_horizon = 5
    
    # Define robots with their start and goal positions
    robots = [
        Robot(1, Position(0, 0), Position(3, 3)),
        Robot(2, Position(3, 0), Position(0, 3))
    ]
    
    # Define obstacles
    obstacles = {Position(1, 1), Position(2, 2)}
    
    # Solve the problem
    paths = solve_warehouse_problem(width, height, robots, obstacles, time_horizon)
    
    if paths:
        print("Solution found!")
        for robot_id, path in paths.items():
            print(f"\nRobot {robot_id} path:")
            for t, pos in enumerate(path):
                print(f"Time {t}: ({pos.x}, {pos.y})")
    else:
        print("No solution exists for the given time horizon.")

if __name__ == "__main__":
    main()
from typing import List, Optional
from grid_types import Grid, GridPosition
from sat_solver import CNFFormula, solve_sat, SATVariable

class PathPlanner:
    def __init__(self, grid: Grid, start: GridPosition, goal: GridPosition, max_steps: int):
        self.grid = grid
        self.start = start
        self.goal = goal
        self.max_steps = max_steps

    def get_pos_var(self, formula: CNFFormula, row: int, col: int, time: int) -> SATVariable:
        """Helper to get variable for a position at a specific time"""
        return formula.get_var(f"pos_{row}_{col}_{time}")

    def create_sat_formula(self) -> CNFFormula:
        formula = CNFFormula()
        
        # 1. Initial state constraints
        # Robot must start at the start position
        start_var = self.get_pos_var(formula, self.start.row, self.start.col, 0)
        formula.add_clause([str(start_var)])
        
        # 2. Goal state constraints
        # Robot must reach goal at some time step t <= max_steps
        goal_vars = []
        for t in range(self.max_steps + 1):
            goal_var = self.get_pos_var(formula, self.goal.row, self.goal.col, t)
            goal_vars.append(str(goal_var))
        # Add clause requiring goal to be reached at least once
        formula.add_clause(goal_vars)
        
        # 3. For each time step
        for t in range(self.max_steps + 1):
            # At least one position must be occupied at each time step
            time_step_vars = []
            
            for row in range(self.grid.rows):
                for col in range(self.grid.cols):
                    if self.grid.is_valid_position(GridPosition(row, col)):
                        pos_var = self.get_pos_var(formula, row, col, t)
                        time_step_vars.append(str(pos_var))
            
            # Must be in at least one position
            formula.add_clause(time_step_vars)
            
            # Cannot be in more than one position
            for i, var1 in enumerate(time_step_vars):
                for var2 in time_step_vars[i + 1:]:
                    formula.add_clause([f"-{var1}", f"-{var2}"])
        
        # 4. Movement constraints
        for t in range(self.max_steps):
            for row in range(self.grid.rows):
                for col in range(self.grid.cols):
                    if not self.grid.is_valid_position(GridPosition(row, col)):
                        continue
                        
                    current_pos = GridPosition(row, col)
                    current_var = self.get_pos_var(formula, row, col, t)
                    
                    # Get valid neighbors
                    neighbors = self.grid.get_neighbors(current_pos)
                    next_pos_vars = []
                    
                    # For each valid neighbor
                    for next_pos in neighbors:
                        next_var = self.get_pos_var(formula, next_pos.row, next_pos.col, t + 1)
                        next_pos_vars.append(str(next_var))
                    
                    # If we're at current position, must move to one of the neighbors
                    # Also allow staying at goal position once reached
                    if current_pos == self.goal:
                        stay_var = self.get_pos_var(formula, row, col, t + 1)
                        next_pos_vars.append(str(stay_var))
                    
                    if next_pos_vars:
                        formula.add_clause([f"-{current_var}"] + next_pos_vars)
                        
                    # If we move to a neighbor, we must have been at current position
                    for next_var in next_pos_vars:
                        formula.add_clause([f"-{next_var}", str(current_var)])
        
        return formula

    def solve(self) -> Optional[List[GridPosition]]:
        formula = self.create_sat_formula()
        solution = solve_sat(formula)
        
        if solution is None:
            return None
        
        # Extract path from solution
        path = []
        for t in range(self.max_steps + 1):
            for row in range(self.grid.rows):
                for col in range(self.grid.cols):
                    var_name = f"pos_{row}_{col}_{t}"
                    if var_name in formula.var_map:
                        var_index = formula.var_map[var_name]
                        if var_index in solution:
                            pos = GridPosition(row, col)
                            path.append(pos)
                            # If we've reached the goal, we can stop
                            if pos == self.goal:
                                return path[:t+1]  # Return only up to goal
                            break
        return path
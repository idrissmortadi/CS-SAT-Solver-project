# Robot Path Planning in a Warehouse

## Introduction
In modern warehouses, autonomous robots play a crucial role in streamlining logistics by transporting goods efficiently. However, as the number of robots increases, ensuring smooth and collision-free navigation becomes a significant challenge. Poor path planning can lead to congestion, delays, and inefficient warehouse operations, negatively impacting productivity and throughput.

The problem of multi-robot path planning (MRPP) involves computing efficient routes for multiple robots navigating a shared environment while avoiding obstacles and preventing collisions. This task is particularly challenging in warehouse settings, where robots must operate in tight spaces with dynamically changing conditions. Efficient solutions must balance computational feasibility with optimality, ensuring all robots reach their destinations in minimal time while adhering to movement constraints.

To address this, we formulate the multi-robot path planning problem as a Satisfiability (SAT) problem, leveraging the efficiency of modern SAT solvers to compute collision-free paths. By encoding robot movements, obstacle constraints, and avoidance rules into Boolean variables and logical formulas, we can transform path planning into a constraint satisfaction problem solvable using SAT-solving techniques.

## Problem Formulation
### Parameters
We define the warehouse as a **two-dimensional grid** of size $n \times m$, denoted as:
$$ G = \{(x, y) \mid 0 \leq x < n, 0 \leq y < m\} $$
where each cell in $G$ represents a valid position in the warehouse.

### Robots
A set of robots $ R = \{r_1, r_2, \dots, r_k\} $ operates in the grid. Each robot $ r $ has:
- A **start position** $ (x_{\text{start}_r}, y_{\text{start}_r}) $
- A **goal position** $ (x_{\text{goal}_r}, y_{\text{goal}_r}) $

### Time Horizon
We introduce a fixed maximum time $ T $, representing the upper bound on the number of time steps a robot can take to reach its goal. The problem is then solved over a discrete sequence of time steps $ t \in \{0, 1, \dots, T\} $.

### Variables
To model the problem as a SAT instance, we define the following Boolean variables:
- $ P(r, x, y, t) $: **True** if robot $ r $ is at position $ (x, y) $ at time $ t $.
- $ O(x, y) $: **True** if there is an obstacle at position $ (x, y) $.

### Constraints
1. **Movement Restricted to Free Cells:**
   Robots can only move to cells that are not obstacles.
   $$
   \forall x, y \in G, \forall r \in R, \forall t \in [0, T]: O(x, y) \implies \neg P(r, x, y, t)
   $$

2. **Movement to Adjacent Cells Only:**
   Robots can only move to adjacent cells or remain stationary at each time step.
   $$
   \forall r \in R, \forall x, y, t: P(r, x, y, t) \implies \bigvee_{(\Delta x, \Delta y) \in M} P\big(r, \text{clamp}(x+\Delta x, 0, n-1), \text{clamp}(y+\Delta y, 0, m-1), \text{clamp}(t+1, 0, T)\big)
   $$

    * The `clamp(x, 0, n)` function restricts the value of `x` within the inclusive range from 0 to `n`.
    * If `x` is less than 0, the function returns 0. If `x` is greater than `n`, the function returns `n`.
    * Otherwise, it returns `x`.

   Here, $ M = \{(0, 1), (1, 0), (0, 0), (-1, 0), (0, -1)\} $ represents valid movements (up, down, left, right, or staying stationary).

3. **Collision Avoidance:**
   Two robots cannot occupy the same cell at the same time.
   $$
   \forall r, r' \in R, r \neq r', \forall x, y, t: P(r, x, y, t) \implies \neg P(r', x, y, t)
   $$

4. **Position Switching Prohibition:**
   Two robots cannot swap positions between consecutive time steps.
   $$
   \forall r, r' \in R, r \neq r', \forall x, y, t, \forall (\Delta x, \Delta y) \in M \setminus \{(0, 0)\}:
   P(r, x, y, t) \wedge P(r', x+\Delta x, y+\Delta y, t) \implies \neg \big(P(r, x+\Delta x, y+\Delta y, t+1) \wedge P(r', x, y, t+1)\big)
   $$

### Objective
The goal is to compute a valid set of paths for all robots such that:
- All robots reach their designated goals within the time horizon $ T $.
- All constraints are satisfied.
- (Optional) The total path length or time taken is minimized.

## Implementation Details : Detailed Explanation of the `WarehousePathPlanner` Class

### Classes and Data Structures

#### Position
- **Attributes:**
  - `x`: Integer representing the x-coordinate.
  - `y`: Integer representing the y-coordinate.
- **Methods:**
  - `_hash_`: Returns a hash value based on the coordinates.
  - `_eq_`: Checks equality between two `Position` instances.

#### Robot
- **Attributes:**
  - `id`: Unique identifier for the robot.
  - `start`: `Position` object representing the starting position.
  - `goal`: `Position` object representing the goal position.

### WarehousePathPlanner
- **Attributes:**
  - `width`: Width of the warehouse grid.
  - `height`: Height of the warehouse grid.
  - `time_horizon`: Maximum time steps allowed.
  - `cnf`: CNF formula to store constraints.
  - `var_map`: Mapping of variable names to integers.
  - `next_var`: Next available variable number.
  - `moves`: List of valid movements (stay, up, right, down, left).

- **Methods:**
  - `create_variable(robot_id, x, y, t)`: Creates a variable representing a robot's position at a given time.
  - `add_initial_positions(robots)`: Adds constraints for the initial positions of robots.
  - `add_goal_positions(robots)`: Adds constraints for the goal positions of robots.
  - `add_obstacle_constraints(obstacles, robots)`: Adds constraints to prevent robots from occupying obstacle positions.
  - `add_movement_constraints(robots)`: Adds constraints for valid movements between time steps.
  - `add_collision_avoidance(robots)`: Adds constraints to prevent robots from occupying the same position.
  - `add_position_switching_prohibition(robots)`: Adds constraints to prevent robots from switching positions.
  - `add_implication(antecedent, consequents)`: Adds implication constraints to the CNF formula.
  - `solve()`: Solves the SAT problem and returns the solution if one exists.
  - `decode_solution(solution)`: Converts the SAT solution to robot paths.

### Code Commentary

#### Imports and Data Structures

```python
import itertools
from dataclasses import dataclass
from typing import Dict, List, Optional, Set

from pysat.formula import CNF
from pysat.solvers import Glucose3
```
- **Imports:**
  - `itertools`: Provides functions for efficient looping, useful for generating combinations.
  - `dataclasses`: Simplifies the creation of classes that primarily store data.
  - `typing`: Provides support for type hints, enhancing code readability and maintainability.
  - `pysat.formula` and `pysat.solvers`: Used for creating and solving SAT problems.

#### Position Class

```python
@dataclass(frozen=True)
class Position:
    x: int
    y: int

    def _hash_(self):
        return hash((self.x, self.y))

    def _eq_(self, other):
        if not isinstance(other, Position):
            return NotImplemented
        return self.x == other.x and self.y == other.y
```
- **Position Class:**
  - Represents a position in the grid with `x` and `y` coordinates.
  - `frozen=True`: Makes instances immutable and hashable, allowing them to be used as keys in dictionaries.
  - `_hash_` and `_eq_`: Methods to enable comparison and hashing of `Position` objects.

#### Robot Class

```python
@dataclass
class Robot:
    id: int
    start: Position
    goal: Position
```
- **Robot Class:**
  - Represents a robot with a unique `id`, a starting `Position`, and a goal `Position`.



The `WarehousePathPlanner` class encapsulates the logic for modeling the multi-robot path planning problem as a SAT instance. At its core, the class builds a Conjunctive Normal Form (CNF) formula representing all the movement, collision, and obstacle constraints that must be satisfied. Here’s a breakdown of the class and its key methods:

### Initialization

```python
def __init__(self, width: int, height: int, time_horizon: int):
    self.width = width
    self.height = height
    self.time_horizon = time_horizon
    self.cnf = CNF()          # The CNF object that will accumulate all SAT clauses.
    self.var_map = {}         # Maps descriptive variable names (e.g., "R1X2Y3T4") to unique integer IDs.
    self.next_var = 1         # The next available variable number (used to ensure each variable is unique).
    # Valid movements: stay in place, move up, right, down, or left.
    self.moves = [(0, 0), (0, 1), (1, 0), (0, -1), (-1, 0)]
```

**Interpretation:**  
- **Dimensions & Time:** The planner knows the grid size and the number of time steps available.
- **CNF & Variable Mapping:** All constraints are added to the `CNF` instance. Each robot’s presence in a cell at a specific time is represented by a unique Boolean variable (managed by `var_map`).
- **Movements:** The list of allowed moves includes both movement and staying in place.

---

### Creating Variables

```python
def create_variable(self, robot_id: int, x: int, y: int, t: int) -> int:
    key = f"R{robot_id}X{x}Y{y}T{t}"
    if key not in self.var_map:
        self.var_map[key] = self.next_var
        self.next_var += 1
    return self.var_map[key]
```

**Interpretation:**  
- This helper function generates a unique variable for each possible configuration: a robot with a given ID at position `(x, y)` at time `t`.  
- The string key (e.g., `"R1X2Y3T4"`) makes it easy to understand which robot and time step the variable represents.  
- Each key is mapped to a unique integer that the SAT solver uses.

---

### Adding Initial Position Constraints

```python
def add_initial_positions(self, robots: List[Robot]):
    for robot in robots:
        start_var = self.create_variable(robot.id, robot.start.x, robot.start.y, 0)
        self.cnf.append([start_var])
        for x in range(self.width):
            for y in range(self.height):
                var = self.create_variable(robot.id, x, y, 0)
                if x != robot.start.x or y != robot.start.y:
                    self.cnf.append([-var])
```

**Interpretation:**  
- **Positive Clause:** For each robot, the variable corresponding to its starting position at time `0` is added as a positive clause (forcing the robot to be at that position initially).  
- **"At Most One" Constraint:** For the starting time step, all other positions are negated to ensure that the robot is not placed in any cell other than its starting position.

---

### Adding Goal Position Constraints

```python
def add_goal_positions(self, robots: List[Robot]):
    for robot in robots:
        goal_var = self.create_variable(robot.id, robot.goal.x, robot.goal.y, self.time_horizon)
        self.cnf.append([goal_var])
        for x in range(self.width):
            for y in range(self.height):
                if x != robot.goal.x or y != robot.goal.y:
                    var = self.create_variable(robot.id, x, y, self.time_horizon)
                    self.cnf.append([-var])
```

**Interpretation:**  
- **Goal Enforcement:** Similar to initial positions, a positive clause is added to ensure that each robot is at its designated goal at the final time step.  
- **Uniqueness:** The robot is prohibited from being anywhere else at time `T`, enforcing that the goal is the unique terminal position.

---

### Adding Obstacle Constraints

```python
def add_obstacle_constraints(self, obstacles: Set[Position], robots: List[Robot]):
    for obstacle in obstacles:
        if not (0 <= obstacle.x < self.width and 0 <= obstacle.y < self.height):
            raise ValueError("Obstacle is outside the warehouse bounds")
        for robot in robots:
            for t in range(self.time_horizon + 1):
                var = self.create_variable(robot.id, obstacle.x, obstacle.y, t)
                self.cnf.append([-var])
```

**Interpretation:**  
- **Obstacle Prohibition:** For each obstacle position, the code iterates over all robots and all time steps, adding a clause that prohibits any robot from being at that cell at any time.  
- This ensures that the generated paths will never cross an obstacle.

---

### Adding Movement Constraints

```python
def add_movement_constraints(self, robots: List[Robot]):
    for robot in robots:
        for t in range(self.time_horizon + 1):
            positions_at_t = []
            for x in range(self.width):
                for y in range(self.height):
                    var = self.create_variable(robot.id, x, y, t)
                    positions_at_t.append(var)
            self.cnf.append(positions_at_t)
            for pos1, pos2 in itertools.combinations(positions_at_t, 2):
                self.cnf.append([-pos1, -pos2])
            if t < self.time_horizon:
                for x in range(self.width):
                    for y in range(self.height):
                        current_pos = self.create_variable(robot.id, x, y, t)
                        next_positions = []
                        for dx, dy in self.moves:
                            next_x, next_y = x + dx, y + dy
                            if 0 <= next_x < self.width and 0 <= next_y < self.height:
                                next_pos = self.create_variable(robot.id, next_x, next_y, t + 1)
                                next_positions.append(next_pos)
                        if next_positions:
                            self.add_implication(current_pos, next_positions)
```

**Interpretation:**  
- **One-Position-Per-Time-Step:**  
  - For every robot at every time step, the planner collects all possible positions and adds a clause ensuring the robot is in at least one of these positions.  
  - It then adds pairwise "at most one" constraints (no two positions can be true at the same time) so that the robot occupies exactly one cell per time step.
- **Valid Movement Enforcement:**  
  - For each cell and time step (except the last), the method computes the set of valid next positions (neighbors including the option to stay in place).
  - An implication is added: if the robot is at a certain position at time `t`, then at time `t+1` it must be in one of the valid adjacent (or same) cells. This keeps the robot's movement realistic and within allowed bounds.

---

### Adding Collision Avoidance Constraints

```python
def add_collision_avoidance(self, robots: List[Robot]):
    for t in range(self.time_horizon + 1):
        for x in range(self.width):
            for y in range(self.height):
                robot_vars = []
                for robot in robots:
                    var = self.create_variable(robot.id, x, y, t)
                    robot_vars.append(var)
                for var1, var2 in itertools.combinations(robot_vars, 2):
                    self.cnf.append([-var1, -var2])
```

**Interpretation:**  
- **Vertex Collision Avoidance:**  
  - For each cell and at each time step, the method gathers all variables representing different robots occupying that cell.
  - It then ensures that at most one robot can be at that cell at the same time by adding pairwise constraints, which prevent any two variables from being true simultaneously.

---

### Adding Position Switching Prohibition

```python
def add_position_switching_prohibition(self, robots: List[Robot]):
    for t in range(self.time_horizon):
        for x1 in range(self.width):
            for y1 in range(self.height):
                for dx, dy in self.moves[1:]:
                    x2, y2 = x1 + dx, y1 + dy
                    if 0 <= x2 < self.width and 0 <= y2 < self.height:
                        for r1, r2 in itertools.combinations(robots, 2):
                            r1_pos1_t = self.create_variable(r1.id, x1, y1, t)
                            r2_pos2_t = self.create_variable(r2.id, x2, y2, t)
                            r1_pos2_t1 = self.create_variable(r1.id, x2, y2, t + 1)
                            r2_pos1_t1 = self.create_variable(r2.id, x1, y1, t + 1)
                            self.cnf.append([-r1_pos1_t, -r2_pos2_t, -r1_pos2_t1, -r2_pos1_t1])
```

**Interpretation:**  
- **Preventing Swapping:**  
  - This method prevents two robots from swapping positions in consecutive time steps, which is a special case of collision where they might "pass through" each other.
  - For each adjacent pair of cells, and for every pair of robots, it adds a constraint that if one robot is at position `(x1, y1)` and another is at `(x2, y2)` at time `t`, then they cannot simply exchange these positions at time `t+1`.
  
---

### Adding Implication Constraints

```python
def add_implication(self, antecedent: int, consequents: List[int]):
    self.cnf.append([-antecedent] + consequents)
```

**Interpretation:**  
- **Implication Clause:**  
  - This is a helper function to express the logical implication: “if the antecedent is true, then at least one of the consequents must be true.”
  - In the context of movement, if a robot is at a given cell at time `t`, then this clause forces the robot to be in one of the valid next positions at time `t+1`.

---

### Solving and Decoding the SAT Problem

```python
def solve(self) -> Optional[Dict[str, bool]]:
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
```

**Interpretation:**  
- **Solving:**  
  - The SAT problem is handed off to the Glucose3 solver.
  - If a solution is found (i.e., a truth assignment for all variables that satisfies all CNF clauses), it returns a dictionary mapping each variable name to a Boolean value.
- **Error Handling:**  
  - The code ensures that a valid model (list of literals) is returned before processing the solution.

```python
def decode_solution(self, solution: Dict[str, bool]) -> Dict[int, List[Position]]:
    paths = {}
    for var_name, is_true in solution.items():
        if is_true:
            numbers = "".join(c if c.isdigit() else " " for c in var_name).split()
            robot_id, x, y, t = map(int, numbers)
            if robot_id not in paths:
                paths[robot_id] = [None] * (self.time_horizon + 1)
            paths[robot_id][t] = Position(x, y)
    for path in paths.values():
        if None in path:
            raise ValueError("Invalid solution: incomplete path detected")
    return paths
```

**Interpretation:**  
- **Solution Decoding:**  
  - After a satisfying model is obtained, this method parses each true variable (e.g., `"R1X2Y3T4"` becomes robot 1 at position `(2, 3)` at time 4).
  - It reconstructs the path for each robot over the time horizon.
- **Completeness Check:**  
  - The method checks that every time step has been assigned a position; if not, it raises an error indicating an incomplete path.

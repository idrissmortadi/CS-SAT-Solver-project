# Multi-Robot Path Planning Problem

## Overview
Multi-Robot Path Planning (MRPP) is a key challenge in robotics where multiple robots must navigate through a shared grid environment to reach their respective goals. This problem considers constraints such as avoiding obstacles, preventing robot collisions, and optimizing the movement across a fixed number of time steps.

The problem has applications in robotics, autonomous vehicle coordination, warehouse management, and game AI.

---

## Problem Statement
Given a 2D grid environment with a fixed number of robots:

1. Each **robot** has:
   - A **start location** (its initial position on the grid).
   - A **goal location** (its target position on the grid).

2. The **grid** is represented as a matrix where:
   - `0` indicates a **free cell** that robots can traverse.
   - `1` indicates a **blocked cell** representing an obstacle.

3. Movements:
   - Robots can move up, down, left, right, or remain stationary in each time step.
   - Each movement takes **1 unit of time**.

4. Constraints:
   - **Obstacle Avoidance**: Robots cannot enter cells with obstacles.
   - **Collision Avoidance**:
     - Robots cannot occupy the same cell at the same time.
     - Robots cannot cross paths, i.e., exchange positions in a single time step.
   - **Fixed Time Horizon**: All robots must reach their goal within a fixed number of time steps \(T\).

5. Output:
   - A sequence of movements for each robot over \(T\) time steps that satisfies the constraints.
   - If no valid solution exists, indicate the problem is unsolvable.

---

## Input Specification
1. **Grid**:
   A matrix of size \(M \times N\) where:
   - `0`: Free cell.
   - `1`: Blocked cell.
   
   Example:
   ```
   0 0 1 0 0
   0 1 1 0 0
   0 0 0 1 0
   1 1 0 1 0
   0 0 0 0 0
   ```

2. **Robots**:
   - Start and goal positions for each robot.
   - Example: 
     ```python
     robots = [
         {"start": (0, 0), "goal": (4, 4)},
         {"start": (1, 1), "goal": (3, 3)}
     ]
     ```

3. **Time Horizon**:
   - Maximum number of time steps \(T\) within which all robots must reach their goals.
   - Example: \(T = 10\).

---

## Output Specification
The solution should provide:
1. **Movements**:
   - A sequence of movements for each robot at every time step.
   - Example:
     ```
     Robot 1: [(0,0), (0,1), (1,1), (2,2), (3,3), (4,4)]
     Robot 2: [(1,1), (1,2), (2,2), (3,3), (3,3), (3,3)]
     ```

2. **Visualization** (Optional):
   - A time-lapse representation of the grid at each time step showing robot movements.

3. **Feasibility**:
   - If no valid solution exists, output: "No valid solution exists."

---

## Challenges
1. **Collision Avoidance**:
   - Preventing two robots from occupying the same cell or crossing paths at the same time step.

2. **Optimization**:
   - Minimize the total time or distance traveled by all robots.

3. **Scalability**:
   - Managing a large number of robots in dense or complex environments.

4. **Dynamic Coordination**:
   - Allowing robots to remain stationary when necessary to avoid conflicts.

---

## Applications
- **Warehouse Automation**: Coordinating robots in storage and retrieval systems.
- **Autonomous Vehicles**: Traffic management for self-driving cars.
- **Game AI**: Coordinating units in strategy games.

---

## Goals
1. Formulate the problem as a **SAT problem** using logical constraints.
2. Implement a solution using a **SAT solver** (e.g., Gophersat).
3. Ensure the following:
   - Each robot reaches its goal without collisions.
   - All constraints are satisfied.

---

## Implementation Details
The solution will involve:
1. **SAT Encoding**:
   - Define variables to represent each robot's position at each time step.
   - Encode constraints for valid movements, collision avoidance, and goal satisfaction.

2. **SAT Solver**:
   - Use Gophersat to find a feasible solution.

3. **Validation**:
   - Verify that the output meets the requirements.
   - Test with multiple grid and robot configurations.

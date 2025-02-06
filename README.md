# Robot Path Planning in a Warehouse

## Introduction
Efficient path planning is a critical challenge in robotics, especially in warehouse environments where multiple robots are tasked with transporting goods from one location to another. The primary objective is to ensure that robots can navigate the warehouse grid effectively, avoiding collisions with obstacles and each other, while reaching their designated destinations in minimal time.

This document outlines a robust approach to multi-robot path planning, formulated as a Satisfiability (SAT) problem, enabling efficient computation of collision-free paths for a set of robots in a fixed grid environment.

## Problem Formulation

### Parameters
- **Grid Dimensions:** The warehouse is represented as a two-dimensional grid of size $ n \times m $. We note $G = \{0,1,\dots,n\} \times \{0,1,\dots,m\}$ the grid of points.
- **Robots:** A set of robots $ R $, each with a unique start position $ (x_{\text{start}_r}, y_{\text{start}_r}) $ and goal position $ (x_{\text{goal}_r}, y_{\text{goal}_r}) $.
- **Time Horizon:** A fixed maximum time $ T $, representing the maximum number of time steps allowed for the robots to reach their destinations.

### Variables
- **Robot Positions:** $ P(r, x, y, t) $: A Boolean variable that is true if robot $ r $ is at position $ (x, y) $ at time $ t $.
- **Obstacles:** $ O(x, y) $: A Boolean variable that is true if an obstacle is present in the coordinates  $ (x, y) $.
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

## Proposed Formulas
Below is a detailed breakdown of the formulas used in the SAT problem formulation:

1. **Obstacle Avoidance:**
   Ensures that no robot can occupy a cell marked as an obstacle.
   $$
   \forall x, y \in G, \forall r \in R, \forall t \in [0, T]: O(x, y) \implies \neg P(r, x, y, t)
   $$

2. **Adjacency Constraint:**
   Guarantees that robots move only to adjacent cells or remain stationary.
   $$
   \forall r \in R, \forall x, y, t: P(r, x, y, t) \implies \bigvee_{(\Delta x, \Delta y) \in M} P\big(r, \text{clamp}(x+\Delta x, 0, n-1), \text{clamp}(y+\Delta y, 0, m-1), \text{clamp}(t+1, 0, T)\big)
   $$

3. **Collision Avoidance:**
   Prevents two robots from being in the same cell at the same time.
   $$
   \forall r, r' \in R, r \neq r', \forall x, y, t: P(r, x, y, t) \implies \neg P(r', x, y, t)
   $$

4. **Position Switching Prohibition:**
   Ensures that two robots cannot exchange positions between consecutive time steps.
   $$
   \forall r, r' \in R, r \neq r', \forall x, y, t, \forall (\Delta x, \Delta y) \in M \setminus \{(0, 0)\}:
   P(r, x, y, t) \wedge P(r', x+\Delta x, y+\Delta y, t) \implies \neg \big(P(r, x+\Delta x, y+\Delta y, t+1) \wedge P(r', x, y, t+1)\big)
   $$

These formulas ensure safe and efficient path planning for multiple robots navigating the warehouse grid. By encoding the problem as a SAT problem, it can be solved using SAT solvers to determine a feasible set of paths that satisfy all constraints.


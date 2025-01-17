def encode_sat(grid, start, goal):
    rows, cols = len(grid), len(grid[0])
    clauses = []

    def var(r, c, t):
        return r * cols + c + t * rows * cols + 1

    # Encode start position at t=0
    clauses.append([var(*start, 0)])

    # Encode goal position at some t (dynamic, up to max steps)
    max_steps = rows * cols
    goal_vars = [var(*goal, t) for t in range(max_steps)]
    clauses.append(goal_vars)

    # Add constraints for valid movements and no revisiting
    for t in range(max_steps):
        for r in range(rows):
            for c in range(cols):
                if grid[r][c] == 1:
                    # Obstacles cannot be visited
                    clauses.append([-var(r, c, t)])
                else:
                    # A cell can only be visited at most once
                    visit_once = [-var(r, c, t_prime) for t_prime in range(max_steps) if t_prime != t]
                    clauses.append([-var(r, c, t)] + visit_once)

                    # Enforce movement to neighbors if the cell is visited
                    if t < max_steps - 1:
                        neighbors = []
                        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                            nr, nc = r + dr, c + dc
                            if 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] == 0:
                                neighbors.append(var(nr, nc, t + 1))
                        # If a cell is visited at time t, it must move to one of its neighbors
                        clauses.append([-var(r, c, t)] + neighbors)

    # Ensure no cell is revisited at any time step
    for r in range(rows):
        for c in range(cols):
            if grid[r][c] == 0:
                visited_once = [-var(r, c, t) for t in range(max_steps)]
                clauses.append(visited_once)

    return clauses




def write_cnf(clauses, file_path):
    with open(file_path, 'w') as f:
        f.write(f"p cnf {max(map(abs, [var for clause in clauses for var in clause]))} {len(clauses)}\n")
        for clause in clauses:
            f.write(' '.join(map(str, clause)) + ' 0\n')

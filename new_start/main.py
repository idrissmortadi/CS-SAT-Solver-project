from input_handler import load_grid, save_result
from sat_formulation import encode_sat, write_cnf
from solver import solve_sat
from visualize import visualize

def extract_path(solution, rows, cols, max_steps):
    path = []
    for var in solution:
        if var > 0:
            var -= 1
            t = var // (rows * cols)
            r = (var % (rows * cols)) // cols
            c = (var % (rows * cols)) % cols
            path.append((t, r, c))
    # Sort by time step and extract grid positions
    path = sorted(path)
    return [(r, c) for _, r, c in path]

def main():
    grid, start, goal = load_grid('grid_input.json')
    clauses = encode_sat(grid, start, goal)
    write_cnf(clauses, 'path_planning.cnf')

    solution = solve_sat('path_planning.cnf')
    if solution is None:
        print("No valid path exists!")
    else:
        rows, cols = len(grid), len(grid[0])
        max_steps = rows * cols
        path = extract_path(solution, rows, cols, max_steps)
        visualize(grid, path)
        save_result(grid, path, 'grid_output.txt')

if __name__ == "__main__":
    main()

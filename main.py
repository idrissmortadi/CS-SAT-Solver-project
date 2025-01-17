from path_planner import PathPlanner
from grid_types import Grid,GridPosition

def main():
    # Example grid from the problem statement
    grid_data = [
        [0, 0, 1, 0, 0],
        [0, 1, 1, 0, 0],
        [0, 0, 0, 1, 0],
        [1, 1, 0, 1, 0],
        [0, 0, 0, 0, 0]
    ]
    
    grid = Grid(grid_data)
    start = GridPosition(0, 0)  # Start at top-left
    goal = GridPosition(2, 0)   # Goal at bottom-right
    
    print("Initial grid:")
    print(grid.visualize_path(start, goal))
    print("\nSearching for path...")
    
    # Try to find a path with maximum 10 steps
    planner = PathPlanner(grid, start, goal, max_steps=10)
    path = planner.solve()
    
    if path is None:
        print("No path found!")
    else:
        print("Path found!")
        print(grid.visualize_path(start, goal, path))
        print("\nPath coordinates:")
        for i, pos in enumerate(path):
            print(f"Step {i}: ({pos.row}, {pos.col})")

if __name__ == "__main__":
    main()
def visualize(grid, path):
    for r, c in path:
        if grid[r][c] == 0:
            grid[r][c] = '*'
    grid[path[0][0]][path[0][1]] = 'S'
    grid[path[-1][0]][path[-1][1]] = 'G'
    for row in grid:
        print(' '.join(map(str, row)))

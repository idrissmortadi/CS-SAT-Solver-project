import json

def load_grid(file_path):
    with open(file_path, 'r') as f:
        data = json.load(f)
    return data['grid'], tuple(data['start']), tuple(data['goal'])

def save_result(grid, path, output_path):
    for r, c in path:
        if grid[r][c] == 0:
            grid[r][c] = '*'
    grid[path[0][0]][path[0][1]] = 'S'
    grid[path[-1][0]][path[-1][1]] = 'G'
    with open(output_path, 'w') as f:
        for row in grid:
            f.write(' '.join(map(str, row)) + '\n')

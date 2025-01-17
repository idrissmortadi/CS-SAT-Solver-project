from typing import List, Optional
from dataclasses import dataclass

@dataclass
class GridPosition:
    row: int
    col: int

    def __hash__(self):
        return hash((self.row, self.col))
    
    def __eq__(self, other):
        if not isinstance(other, GridPosition):
            return False
        return self.row == other.row and self.col == other.col

class Grid:
    def __init__(self, grid_data: List[List[int]]):
        self.data = grid_data
        self.rows = len(grid_data)
        self.cols = len(grid_data[0]) if grid_data else 0

    def is_valid_position(self, pos: GridPosition) -> bool:
        return (0 <= pos.row < self.rows and 
                0 <= pos.col < self.cols and 
                self.data[pos.row][pos.col] == 0)

    def get_neighbors(self, pos: GridPosition) -> List[GridPosition]:
        """Get all valid neighboring positions (up, down, left, right)"""
        possible_moves = [
            GridPosition(pos.row - 1, pos.col),  # up
            GridPosition(pos.row + 1, pos.col),  # down
            GridPosition(pos.row, pos.col - 1),  # left
            GridPosition(pos.row, pos.col + 1),  # right
        ]
        return [move for move in possible_moves if self.is_valid_position(move)]

    def visualize_path(self, start: GridPosition, goal: GridPosition, 
                      path: Optional[List[GridPosition]] = None) -> str:
        """Create a string visualization of the grid with the path"""
        result = []
        for i in range(self.rows):
            row = []
            for j in range(self.cols):
                pos = GridPosition(i, j)
                if pos == start:
                    row.append('S')
                elif pos == goal:
                    row.append('G')
                elif path and pos in path:
                    row.append('*')
                else:
                    row.append(str(self.data[i][j]))
            result.append(' '.join(row))
        return '\n'.join(result)
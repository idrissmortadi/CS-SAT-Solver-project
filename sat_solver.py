import subprocess
from typing import List, Optional, Set, Dict
import tempfile
import os

class SATVariable:
    def __init__(self, name: str, index: int):
        self.name = name
        self.index = index

    def __str__(self):
        return f"{self.index}"

    def __neg__(self):
        return f"-{self.index}"

class CNFFormula:
    def __init__(self):
        self.clauses: List[List[str]] = []
        self.next_index = 1
        self.var_map: Dict[str, int] = {}

    def get_var(self, name: str) -> SATVariable:
        if name not in self.var_map:
            self.var_map[name] = self.next_index
            self.next_index += 1
        return SATVariable(name, self.var_map[name])

    def add_clause(self, clause: List[str]):
        self.clauses.append(clause)

    def to_dimacs(self) -> str:
        """Convert the formula to DIMACS format"""
        lines = [f"p cnf {len(self.var_map)} {len(self.clauses)}"]
        for clause in self.clauses:
            lines.append(' '.join(clause) + ' 0')
        return '\n'.join(lines)

def solve_sat(formula: CNFFormula) -> Optional[Set[int]]:
    """Solve a SAT formula using gophersat"""
    # Write formula to temporary file
    with tempfile.NamedTemporaryFile(mode='w', delete=False) as f:
        f.write(formula.to_dimacs())
        temp_filename = f.name

    try:
        # Run gophersat
        result = subprocess.run(['gophersat_win64.exe', temp_filename], 
                              capture_output=True, text=True)
        
        # Parse the output
        if result.stdout.strip().startswith('SAT'):
            assignment = result.stdout.split('\n')[1].strip().split()[:-1]
            return {int(v) for v in assignment if not v.startswith('-')}
        return None
    finally:
        os.unlink(temp_filename)
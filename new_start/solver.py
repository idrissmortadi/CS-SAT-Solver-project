import subprocess

def solve_sat(cnf_file):
    result = subprocess.run(['gophersat_win64.exe', cnf_file], capture_output=True, text=True)
    if "UNSAT" in result.stdout:
        return None
    lines = result.stdout.splitlines()
    for line in lines:
        if line.startswith("v "):
            return list(map(int, line[2:].split()))
    return None

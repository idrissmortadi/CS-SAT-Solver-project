class SATFileUtils:
    def __init__(self, file_path):
        self.file_path = file_path
        self.clauses = []

    def read_cnf(self):
        """Reads a CNF file and stores clauses."""
        with open(self.file_path, "r") as file:
            lines = file.readlines()

        self.clauses = []
        for line in lines:
            if line.startswith("p") or line.startswith("c"):
                continue  # Skip header and comments
            clause = list(map(int, line.split()))
            if clause[-1] == 0:
                clause.pop()  # Remove trailing zero (end of clause marker)
            self.clauses.append(clause)

    def write_cnf(self, output_path):
        """Writes clauses to a CNF file."""
        with open(output_path, "w") as file:
            # Writing header: p cnf <num_variables> <num_clauses>
            num_vars = max(
                abs(literal) for clause in self.clauses for literal in clause
            )
            num_clauses = len(self.clauses)
            file.write(f"p cnf {num_vars} {num_clauses}\n")

            # Writing clauses
            for clause in self.clauses:
                file.write(" ".join(map(str, clause)) + " 0\n")

    def read_wcnf(self):
        """Reads a WCNF file."""
        with open(self.file_path, "r") as file:
            lines = file.readlines()

        self.clauses = []
        for line in lines:
            if line.startswith("c") or line.startswith("p"):
                continue  # Skip comments and header
            parts = list(map(int, line.split()))
            weight = parts[0]
            clause = parts[1:]
            if clause[-1] == 0:
                clause.pop()  # Remove trailing zero
            self.clauses.append((weight, clause))

    def write_wcnf(self, output_path):
        """Writes clauses to a WCNF file."""
        with open(output_path, "w") as file:
            # Writing header: p wcnf <num_variables> <num_clauses>
            num_vars = max(
                abs(literal) for weight, clause in self.clauses for literal in clause
            )
            num_clauses = len(self.clauses)
            file.write(f"p wcnf {num_vars} {num_clauses}\n")

            # Writing clauses with weights
            for weight, clause in self.clauses:
                file.write(f"{weight} " + " ".join(map(str, clause)) + " 0\n")

    def read_opb(self):
        """Reads a OPB file (Pseudo-Boolean)."""
        with open(self.file_path, "r") as file:
            lines = file.readlines()

        self.clauses = []
        for line in lines:
            if line.startswith("c") or line.startswith("p"):
                continue  # Skip comments and header
            parts = list(map(int, line.split()))
            # OPB uses sum of literals with weights and inequality sign
            self.clauses.append(parts)

    def write_opb(self, output_path):
        """Writes clauses to a OPB file."""
        with open(output_path, "w") as file:
            # Writing a generic header for OPB
            num_vars = max(
                abs(literal) for clause in self.clauses for literal in clause
            )
            file.write(f"p opb {num_vars}\n")

            # Writing clauses
            for clause in self.clauses:
                file.write(" ".join(map(str, clause)) + "\n")

    def add_clause(self, clause):
        """Adds a clause to the current list of clauses."""
        self.clauses.append(clause)

    def remove_clause(self, clause):
        """Removes a clause from the current list of clauses."""
        if clause in self.clauses:
            self.clauses.remove(clause)

    def get_clauses(self):
        """Returns the list of clauses."""
        return self.clauses


# Example usage:
if __name__ == "__main__":
    # CNF Example:
    cnf = SATFileUtils("example.cnf")
    cnf.read_cnf()
    cnf.add_clause([1, -2, 3])
    cnf.write_cnf("new_example.cnf")

    # WCNF Example:
    wcnf = SATFileUtils("example.wcnf")
    wcnf.read_wcnf()
    wcnf.add_clause((3, [1, -2, 3]))  # Weight 3 for clause [1, -2, 3]
    wcnf.write_wcnf("new_example.wcnf")

    # OPB Example:
    opb = SATFileUtils("example.opb")
    opb.read_opb()
    opb.add_clause([2, -1, 4, 5, 3, 6, 0])  # A pseudo-boolean clause
    opb.write_opb("new_example.opb")

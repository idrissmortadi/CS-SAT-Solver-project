import unittest
import os
from tempfile import NamedTemporaryFile
from src.sat_file_utils import SATFileUtils


class TestSATFileUtils(unittest.TestCase):
    def setUp(self):
        """Create temporary files for testing and initialize utility."""
        # CNF example file content
        self.cnf_content = """p cnf 3 3
            1 -2 3 0
            -1 2 3 0
            1 2 -3 0
            """
        # WCNF example file content (with weights)
        self.wcnf_content = """p wcnf 3 3
            5 1 -2 3 0
            3 -1 2 3 0
            2 1 2 -3 0
            """
        # OPB example file content
        self.opb_content = """p opb 3 3
            2 1 2 3 >= 1
            3 1 -2 3 >= 2
            4 2 -1 3 >= 3
            """

        # Create temporary CNF, WCNF, and OPB files
        self.cnf_file = NamedTemporaryFile(delete=False)
        self.cnf_file.write(self.cnf_content.encode())
        self.cnf_file.close()

        self.wcnf_file = NamedTemporaryFile(delete=False)
        self.wcnf_file.write(self.wcnf_content.encode())
        self.wcnf_file.close()

        self.opb_file = NamedTemporaryFile(delete=False)
        self.opb_file.write(self.opb_content.encode())
        self.opb_file.close()

        # Initialize SATFileUtils with the file paths
        self.cnf_util = SATFileUtils(self.cnf_file.name)
        self.wcnf_util = SATFileUtils(self.wcnf_file.name)
        self.opb_util = SATFileUtils(self.opb_file.name)

    def tearDown(self):
        """Clean up temporary files after tests."""
        os.remove(self.cnf_file.name)
        os.remove(self.wcnf_file.name)
        os.remove(self.opb_file.name)

    def test_read_cnf(self):
        """Test reading of CNF file."""
        self.cnf_util.read_cnf()
        clauses = self.cnf_util.get_clauses()
        expected_clauses = [[1, -2, 3], [-1, 2, 3], [1, 2, -3]]
        self.assertEqual(clauses, expected_clauses)

    def test_write_cnf(self):
        """Test writing CNF file."""
        self.cnf_util.read_cnf()
        self.cnf_util.add_clause([2, -3, 1])
        temp_output = NamedTemporaryFile(delete=False)
        self.cnf_util.write_cnf(temp_output.name)

        # Read the written file and verify contents
        with open(temp_output.name, "r") as file:
            content = file.read()
            self.assertIn("p cnf", content)
            self.assertIn("2 -3 1 0", content)

        os.remove(temp_output.name)

    def test_read_wcnf(self):
        """Test reading of WCNF file."""
        self.wcnf_util.read_wcnf()
        clauses = self.wcnf_util.get_clauses()
        expected_clauses = [(5, [1, -2, 3]), (3, [-1, 2, 3]), (2, [1, 2, -3])]
        self.assertEqual(clauses, expected_clauses)

    def test_write_wcnf(self):
        """Test writing WCNF file."""
        self.wcnf_util.read_wcnf()
        self.wcnf_util.add_clause((4, [1, -3, 2]))
        temp_output = NamedTemporaryFile(delete=False)
        self.wcnf_util.write_wcnf(temp_output.name)

        # Read the written file and verify contents
        with open(temp_output.name, "r") as file:
            content = file.read()
            self.assertIn("p wcnf", content)
            self.assertIn("4 1 -3 2 0", content)

        os.remove(temp_output.name)

    def test_read_opb(self):
        """Test reading of OPB file."""
        self.opb_util.read_opb()
        clauses = self.opb_util.get_clauses()
        expected_clauses = [[2, 1, 2, 3, 0], [3, 1, -2, 3, 0], [4, 2, -1, 3, 0]]
        self.assertEqual(clauses, expected_clauses)

    def test_write_opb(self):
        """Test writing OPB file."""
        self.opb_util.read_opb()
        self.opb_util.add_clause([5, -1, 2, 3, 0])
        temp_output = NamedTemporaryFile(delete=False)
        self.opb_util.write_opb(temp_output.name)

        # Read the written file and verify contents
        with open(temp_output.name, "r") as file:
            content = file.read()
            self.assertIn("p opb", content)
            self.assertIn("5 -1 2 3 0", content)

        os.remove(temp_output.name)

    def test_add_clause(self):
        """Test adding a new clause."""
        self.cnf_util.read_cnf()
        self.cnf_util.add_clause([3, -1])
        clauses = self.cnf_util.get_clauses()
        self.assertIn([3, -1], clauses)

    def test_remove_clause(self):
        """Test removing an existing clause."""
        self.cnf_util.read_cnf()
        self.cnf_util.remove_clause([1, -2, 3])
        clauses = self.cnf_util.get_clauses()
        self.assertNotIn([1, -2, 3], clauses)

    def test_invalid_file_format(self):
        """Test handling invalid file formats."""
        invalid_file = NamedTemporaryFile(delete=False)
        invalid_file.write(b"invalid content")
        invalid_file.close()

        invalid_util = SATFileUtils(invalid_file.name)
        with self.assertRaises(ValueError):
            invalid_util.read_cnf()  # Should raise an exception due to invalid CNF format

        os.remove(invalid_file.name)

    def test_empty_file(self):
        """Test handling of empty files."""
        empty_file = NamedTemporaryFile(delete=False)
        empty_file.write(b"")
        empty_file.close()

        empty_util = SATFileUtils(empty_file.name)
        empty_util.read_cnf()  # Should handle gracefully, even though there are no clauses
        self.assertEqual(empty_util.get_clauses(), [])

        os.remove(empty_file.name)

    def test_edge_case_empty_clause(self):
        """Test handling empty clauses in CNF files."""
        edge_case_file = NamedTemporaryFile(delete=False)
        edge_case_file.write(b"p cnf 3 1\n0\n")  # CNF file with an empty clause
        edge_case_file.close()

        edge_case_util = SATFileUtils(edge_case_file.name)
        edge_case_util.read_cnf()
        clauses = edge_case_util.get_clauses()
        self.assertEqual(clauses, [])  # No clauses should be added

        os.remove(edge_case_file.name)


if __name__ == "__main__":
    unittest.main()

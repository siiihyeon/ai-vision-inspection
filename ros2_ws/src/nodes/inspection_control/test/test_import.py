import unittest


class ImportTests(unittest.TestCase):
    def test_control_domain_imports(self) -> None:
        from inspection_control.control_node import ControlNode

        self.assertEqual(ControlNode.__name__, "ControlNode")

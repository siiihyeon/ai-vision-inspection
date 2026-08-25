import unittest


class ImportTests(unittest.TestCase):
    def test_common_contract_imports(self) -> None:
        from inspection_common import NodeId

        self.assertEqual(NodeId.VISION.value, "vision")

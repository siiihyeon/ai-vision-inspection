import unittest


class ImportTests(unittest.TestCase):
    def test_bringup_package_imports(self) -> None:
        import inspection_bringup

        self.assertEqual(inspection_bringup.__name__, "inspection_bringup")

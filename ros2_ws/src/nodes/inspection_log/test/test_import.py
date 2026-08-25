import unittest


class ImportTests(unittest.TestCase):
    def test_log_storage_imports(self) -> None:
        from inspection_log.storage import LogRepository

        self.assertEqual(LogRepository.__name__, "LogRepository")

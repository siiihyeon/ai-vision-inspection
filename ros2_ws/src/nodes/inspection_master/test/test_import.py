import unittest


class ImportTests(unittest.TestCase):
    def test_master_domain_imports(self) -> None:
        from inspection_master.product_flow import ProductLedger

        self.assertEqual(ProductLedger.__name__, "ProductLedger")

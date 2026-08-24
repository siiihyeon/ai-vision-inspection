import unittest


class ImportTests(unittest.TestCase):
    def test_vision_hardware_and_queue_modules_import(self) -> None:
        from inspection_vision.hikrobot_mvs import HikrobotMvsCaptureBackend
        from inspection_vision.inference_queue import InferenceQueue

        self.assertEqual(
            HikrobotMvsCaptureBackend.__name__, "HikrobotMvsCaptureBackend"
        )
        self.assertEqual(InferenceQueue(capacity=1).capacity, 1)

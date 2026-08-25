import unittest

from inspection_control.mega_protocol import decode_frame, encode_frame, parse_event


class MegaProtocolTest(unittest.TestCase):
    def test_frame_round_trip(self):
        frame = encode_frame("C", 7, "POSITION", 1, 1200)
        self.assertEqual(decode_frame(frame), ["C", "7", "POSITION", "1", "1200"])

    def test_bad_crc_is_rejected(self):
        frame = encode_frame("E", "SENSOR", "SENSOR_1", 1, 4, 0)
        corrupted = frame.replace(b"SENSOR_1", b"SENSOR_2")
        self.assertIsNone(decode_frame(corrupted))

    def test_sensor_event_is_parsed(self):
        event = parse_event(["E", "SENSOR", "SENSOR_3", "1", "9", "0"])
        self.assertEqual(event.kind, "SENSOR")
        self.assertEqual(event.values[0], "SENSOR_3")


if __name__ == "__main__":
    unittest.main()
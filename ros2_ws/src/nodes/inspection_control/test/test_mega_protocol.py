import unittest

from inspection_control.mega_protocol import (
    decode_frame,
    decode_frame_diagnostic,
    encode_frame,
    parse_event,
    parse_sensor3_telemetry,
)


class MegaProtocolTest(unittest.TestCase):
    def test_frame_round_trip(self):
        frame = encode_frame("C", 7, "POSITION", 1, 1200)
        self.assertEqual(decode_frame(frame), ["C", "7", "POSITION", "1", "1200"])

    def test_bad_crc_is_rejected(self):
        frame = encode_frame("E", "SENSOR", "SENSOR_1", 1, 4, 0)
        corrupted = frame.replace(b"SENSOR_1", b"SENSOR_2")
        self.assertIsNone(decode_frame(corrupted))
        self.assertEqual(
            decode_frame_diagnostic(corrupted).rejection_reason,
            "crc_mismatch",
        )

    def test_decode_diagnostics_distinguish_failure_categories(self):
        self.assertEqual(
            decode_frame_diagnostic(b"not-a-frame\n").rejection_reason,
            "missing_crc_field",
        )
        self.assertEqual(
            decode_frame_diagnostic(b"body|not-hex\n").rejection_reason,
            "invalid_crc_text",
        )
        self.assertEqual(
            decode_frame_diagnostic(b"\xff\n").rejection_reason,
            "ascii_decode_error",
        )

    def test_sensor_event_is_parsed(self):
        event = parse_event(["E", "SENSOR", "SENSOR_3", "1", "9", "0"])
        self.assertEqual(event.kind, "SENSOR")
        self.assertEqual(event.values[0], "SENSOR_3")

    def test_sensor3_telemetry_is_parsed(self):
        frame = encode_frame(
            "LOG",
            "SENSOR3",
            "RELEASE_CHECK",
            "millis=1234",
            "micros=1234567",
            "distanceCm=15.25",
            "armed=0",
            "detectCount=0",
            "releaseCount=1",
        )
        telemetry = parse_sensor3_telemetry(decode_frame(frame))
        self.assertEqual(telemetry.event, "RELEASE_CHECK")
        self.assertEqual(telemetry.firmware_millis, 1234)
        self.assertAlmostEqual(telemetry.distance_cm, 15.25)
        self.assertFalse(telemetry.detection_armed)
        self.assertEqual(telemetry.consecutive_release_count, 1)

    def test_malformed_sensor3_telemetry_is_rejected(self):
        self.assertIsNone(
            parse_sensor3_telemetry(
                [
                    "LOG",
                    "SENSOR3",
                    "TIMEOUT",
                    "millis=1",
                    "micros=2",
                    "distanceCm=-1.0",
                    "armed=yes",
                    "detectCount=0",
                    "releaseCount=1",
                ]
            )
        )


if __name__ == "__main__":
    unittest.main()

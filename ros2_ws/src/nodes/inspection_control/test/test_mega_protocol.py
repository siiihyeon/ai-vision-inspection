import unittest

from inspection_control.mega_protocol import (
    decode_frame,
    decode_frame_diagnostic,
    encode_frame,
    parse_event,
    parse_sensor_diagnostic,
    parse_sensor_distance,
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

    def test_sensor_distance_event_is_parsed(self):
        frame = encode_frame(
            "E", "SENSOR_DISTANCE", "SENSOR_2", 42, "10.25", "10.50", "10.75"
        )
        event = parse_sensor_distance(decode_frame(frame))
        self.assertEqual(event.sensor_id, "SENSOR_2")
        self.assertEqual(event.sensor_sequence, 42)
        self.assertEqual(event.distances_cm, (10.25, 10.5, 10.75))

    def test_malformed_sensor_distance_event_is_rejected(self):
        invalid_frames = (
            ["E", "SENSOR_DISTANCE", "SENSOR_3", "1", "1", "2", "3"],
            ["E", "SENSOR_DISTANCE", "SENSOR_1", "-1", "1", "2", "3"],
            ["E", "SENSOR_DISTANCE", "SENSOR_1", "1", "1", "nan", "3"],
            ["E", "SENSOR_DISTANCE", "SENSOR_1", "1", "1", "2"],
        )
        for fields in invalid_frames:
            with self.subTest(fields=fields):
                self.assertIsNone(parse_sensor_distance(fields))

    def test_sensor_diagnostic_event_is_parsed(self):
        frame = encode_frame(
            "E",
            "SENSOR_DIAGNOSTIC",
            "SENSOR_1",
            "DETECTION_DROPPED",
            12,
            1234,
            1234567,
            "8.75",
            1,
            5,
            0,
            3,
            1,
            "CONVEYOR_STOPPED",
        )
        event = parse_sensor_diagnostic(decode_frame(frame))
        self.assertEqual(event.sensor_id, "SENSOR_1")
        self.assertEqual(event.event, "DETECTION_DROPPED")
        self.assertEqual(event.sensor_sequence, 12)
        self.assertEqual(event.firmware_millis, 1234)
        self.assertEqual(event.firmware_micros, 1234567)
        self.assertAlmostEqual(event.distance_cm, 8.75)
        self.assertEqual(event.consecutive_detect_count, 5)
        self.assertEqual(event.conveyor_state, 3)
        self.assertTrue(event.pending_detection)
        self.assertEqual(event.reason, "CONVEYOR_STOPPED")

    def test_malformed_sensor_diagnostic_event_is_rejected(self):
        valid = [
            "E", "SENSOR_DIAGNOSTIC", "SENSOR_2", "EVENT_SENT",
            "3", "1234", "1234567", "9.5", "0", "5", "0", "0", "0",
            "RUNNING",
        ]
        invalid_frames = (
            valid[:-1],
            [*valid[:2], "SENSOR_3", *valid[3:]],
            [*valid[:3], "UNKNOWN", *valid[4:]],
            [*valid[:5], "-1", *valid[6:]],
            [*valid[:7], "nan", *valid[8:]],
            [*valid[:12], "2", *valid[13:]],
            [*valid[:13], "not-lowercase"],
        )
        for fields in invalid_frames:
            with self.subTest(fields=fields):
                self.assertIsNone(parse_sensor_diagnostic(fields))

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

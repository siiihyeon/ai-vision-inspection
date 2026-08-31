import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from inspection_control.control_node import ControlNode


def _control(**overrides) -> SimpleNamespace:
    control = SimpleNamespace(
        _mega_lock=threading.RLock(),
        _pending_run_acks={},
        _publish_conveyor_resumed=Mock(),
    )
    for key, value in overrides.items():
        setattr(control, key, value)
    return control


class RunAckConveyorResumedTests(unittest.TestCase):
    """Mega RUN ACK -> ConveyorResumed 승격 경로. 확정 이벤트라 재가동 확인이
    EquipmentState의 depth 유실과 무관하게 도착해야 하는 근거입니다."""

    def test_ok_ack_publishes_conveyor_resumed_and_clears_pending_entry(self) -> None:
        control = _control(_pending_run_acks={7: (1, time.monotonic())})

        ControlNode._handle_run_ack(control, "7", True)

        control._publish_conveyor_resumed.assert_called_once_with(1)
        self.assertNotIn(7, control._pending_run_acks)

    def test_error_ack_clears_pending_entry_without_publishing(self) -> None:
        control = _control(_pending_run_acks={7: (1, time.monotonic())})

        ControlNode._handle_run_ack(control, "7", False)

        control._publish_conveyor_resumed.assert_not_called()
        self.assertNotIn(7, control._pending_run_acks)

    def test_ack_for_untracked_sequence_is_ignored(self) -> None:
        # STOP/SET_OFFSET/HELLO 같은 다른 명령의 ACK은 여기 등록되지 않으므로
        # 조용히 무시해야 한다.
        control = _control(_pending_run_acks={})

        ControlNode._handle_run_ack(control, "99", True)

        control._publish_conveyor_resumed.assert_not_called()

    def test_non_numeric_sequence_is_ignored(self) -> None:
        control = _control(_pending_run_acks={7: (1, time.monotonic())})

        ControlNode._handle_run_ack(control, "not-a-number", True)

        control._publish_conveyor_resumed.assert_not_called()
        self.assertIn(7, control._pending_run_acks)


if __name__ == "__main__":
    unittest.main()

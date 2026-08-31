import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from inspection_common import ConveyorId, SystemState
from inspection_master.master_node import MasterNode


def _equipment_state_master(**overrides) -> SimpleNamespace:
    master = SimpleNamespace(
        session_id="session-1",
        system_state=SystemState.RUN_SYS,
        _pending_run_confirmation=False,
        equipment=SimpleNamespace(
            all_conveyors_running=Mock(return_value=False),
            all_conveyors_stopped=Mock(return_value=False),
        ),
        update_equipment_snapshot=Mock(),
        _confirm_pending_resume=Mock(),
        confirm_all_conveyors_running=Mock(),
        confirm_all_conveyors_stopped=Mock(),
    )
    for key, value in overrides.items():
        setattr(master, key, value)
    return master


def _equipment_state_message(**overrides) -> SimpleNamespace:
    message = SimpleNamespace(
        header=SimpleNamespace(session_id="session-1"),
        upper_running=False,
        upper_stopped=False,
        lower_running=False,
        lower_stopped=False,
        sensor_1_clear=True,
        sensor_2_clear=True,
        sensor_3_clear=True,
        actuator_safe=True,
    )
    for key, value in overrides.items():
        setattr(message, key, value)
    return message


class EquipmentStateLevelConfirmationTests(unittest.TestCase):
    """depth=1 EquipmentState 유실 시 재가동 확인이 영영 안 오던 회귀를 막습니다."""

    def test_running_true_confirms_even_without_a_prior_stopped_sample(self) -> None:
        # 이전엔 "정지 -> 가동" 전이 샘플에만 반응했다(rising edge). depth=1
        # 큐에서 그 전이 샘플 자체가 구독 콜백에 도달하지 못하면, Master가
        # 처음 받는 샘플이 이미 running=True뿐이어도 확인을 놓쳤다.
        master = _equipment_state_master()
        message = _equipment_state_message(upper_running=True, lower_running=False)

        MasterNode._handle_equipment_state(master, message)

        master._confirm_pending_resume.assert_any_call(ConveyorId.UPPER)

    def test_running_true_confirms_on_every_sample_not_just_the_edge(self) -> None:
        # confirm_conveyor_resumed가 RESUME_PENDING이 아닌 cycle은 조용히
        # 무시하는 멱등 호출이므로, 같은 running=True를 반복 수신해도 안전해야
        # 한다.
        master = _equipment_state_master()
        message = _equipment_state_message(upper_running=True, lower_running=False)

        MasterNode._handle_equipment_state(master, message)
        MasterNode._handle_equipment_state(master, message)

        self.assertEqual(master._confirm_pending_resume.call_count, 2)

    def test_running_false_does_not_confirm(self) -> None:
        master = _equipment_state_master()
        message = _equipment_state_message(upper_running=False, lower_running=False)

        MasterNode._handle_equipment_state(master, message)

        master._confirm_pending_resume.assert_not_called()

    def test_session_mismatch_is_ignored(self) -> None:
        master = _equipment_state_master(session_id="session-1")
        message = _equipment_state_message(
            header=SimpleNamespace(session_id="stale-session"),
            upper_running=True,
        )

        MasterNode._handle_equipment_state(master, message)

        master.update_equipment_snapshot.assert_not_called()
        master._confirm_pending_resume.assert_not_called()


class ConveyorResumedEventTests(unittest.TestCase):
    """Mega RUN ACK 기반 확정 이벤트 경로. EquipmentState 유실과 무관합니다."""

    def _master(self, **overrides) -> SimpleNamespace:
        master = SimpleNamespace(
            session_id="session-1",
            _confirm_pending_resume=Mock(),
            _fault_stop=Mock(),
        )
        for key, value in overrides.items():
            setattr(master, key, value)
        return master

    def _message(self, conveyor_id: int, session_id: str = "session-1") -> SimpleNamespace:
        return SimpleNamespace(
            header=SimpleNamespace(session_id=session_id),
            conveyor_id=conveyor_id,
        )

    def test_valid_conveyor_id_confirms_pending_resume(self) -> None:
        master = self._master()

        MasterNode._handle_conveyor_resumed(master, self._message(int(ConveyorId.UPPER)))

        master._confirm_pending_resume.assert_called_once_with(ConveyorId.UPPER)
        master._fault_stop.assert_not_called()

    def test_invalid_conveyor_id_faults_instead_of_confirming(self) -> None:
        master = self._master()

        MasterNode._handle_conveyor_resumed(master, self._message(99))

        master._confirm_pending_resume.assert_not_called()
        master._fault_stop.assert_called_once()

    def test_session_mismatch_is_ignored(self) -> None:
        master = self._master(session_id="session-1")

        MasterNode._handle_conveyor_resumed(
            master, self._message(int(ConveyorId.LOWER), session_id="stale-session")
        )

        master._confirm_pending_resume.assert_not_called()
        master._fault_stop.assert_not_called()


if __name__ == "__main__":
    unittest.main()

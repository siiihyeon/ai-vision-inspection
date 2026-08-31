import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from inspection_common import StationId, SystemState
from inspection_master.master_node import MasterNode
from inspection_master.operation_runtime import StationCyclePhase


class StationStartDeferralTests(unittest.TestCase):
    def test_active_owner_defers_next_product_for_both_stations_and_phases(self) -> None:
        for station_id in (StationId.A, StationId.B):
            for phase in StationCyclePhase:
                with self.subTest(station_id=station_id, phase=phase):
                    emit_log_event = Mock()
                    master = SimpleNamespace(
                        system_state=SystemState.RUN_SYS,
                        _station_cycles={
                            station_id: SimpleNamespace(
                                product_id="owner",
                                phase=phase,
                            )
                        },
                        _deferred_station_starts=set(),
                        _emit_log_event=emit_log_event,
                        _fault_stop=Mock(),
                    )

                    MasterNode._start_station_cycle(master, "next", 2, station_id)

                    self.assertEqual(
                        master._deferred_station_starts,
                        {("next", station_id)},
                    )
                    master._fault_stop.assert_not_called()
                    self.assertEqual(
                        emit_log_event.call_args.kwargs["event_type"],
                        "STATION_START_DEFERRED_UNTIL_RESUME",
                    )
                    self.assertEqual(
                        emit_log_event.call_args.kwargs["payload"]["owning_phase"],
                        phase.value,
                    )

    def test_same_product_does_not_enqueue_duplicate_station_start(self) -> None:
        for station_id in (StationId.A, StationId.B):
            with self.subTest(station_id=station_id):
                emit_log_event = Mock()
                master = SimpleNamespace(
                    system_state=SystemState.RUN_SYS,
                    _station_cycles={
                        station_id: SimpleNamespace(
                            product_id="owner",
                            phase=StationCyclePhase.RESUME_PENDING,
                        )
                    },
                    _deferred_station_starts=set(),
                    _emit_log_event=emit_log_event,
                    _fault_stop=Mock(),
                )

                MasterNode._start_station_cycle(master, "owner", 1, station_id)

                self.assertFalse(master._deferred_station_starts)
                master._fault_stop.assert_not_called()
                emit_log_event.assert_not_called()

    def test_oldest_deferred_product_starts_first(self) -> None:
        station_id = StationId.B
        contexts = {
            "product-3": SimpleNamespace(
                product_id="product-3", fifo_sequence=3, removed=False
            ),
            "product-2": SimpleNamespace(
                product_id="product-2", fifo_sequence=2, removed=False
            ),
        }
        start_station_cycle = Mock()
        master = SimpleNamespace(
            system_state=SystemState.RUN_SYS,
            _deferred_station_starts={
                ("product-3", station_id),
                ("product-2", station_id),
            },
            ledger=SimpleNamespace(get_by_id=contexts.get),
            _start_station_cycle=start_station_cycle,
        )

        MasterNode._start_next_deferred_station_cycle(master, station_id)

        start_station_cycle.assert_called_once_with("product-2", 2, station_id)


if __name__ == "__main__":
    unittest.main()

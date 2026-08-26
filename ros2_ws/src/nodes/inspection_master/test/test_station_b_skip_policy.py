import unittest
import threading
from types import SimpleNamespace
from unittest.mock import Mock

from inspection_master.master_node import MasterNode
from inspection_master.product_flow import ProductPhysicalState


class StationBSkipPolicyTests(unittest.TestCase):
    def test_station_b_is_retained_when_skip_policy_is_disabled(self) -> None:
        emit_log_event = Mock()
        context = SimpleNamespace(
            product_id="product-1",
            snapshot=lambda: {"station_b_skip_requested": False},
        )
        master = SimpleNamespace(
            skip_station_b_after_station_a_ng=False,
            _emit_log_event=emit_log_event,
        )

        MasterNode._cancel_station_b_after_a_terminal(
            master, context, "Station A terminal NG"
        )

        emit_log_event.assert_called_once()
        call = emit_log_event.call_args.kwargs
        self.assertEqual(
            call["event_type"], "STATION_B_RETAINED_AFTER_A_TERMINAL_NG"
        )
        self.assertEqual(call["product_id"], "product-1")
        self.assertFalse(call["payload"]["skip_after_station_a_ng"])

    def test_station_b_is_skipped_when_skip_policy_is_enabled(self) -> None:
        context = SimpleNamespace(
            product_id="product-1",
            physical_state=ProductPhysicalState.FLIPPING,
            request_station_b_skip=Mock(return_value=True),
        )
        publish_cancellation = Mock()
        master = SimpleNamespace(
            skip_station_b_after_station_a_ng=True,
            _flow_lock=threading.RLock(),
            _publish_inference_cancellation=publish_cancellation,
            _station_cycles={},
        )

        MasterNode._cancel_station_b_after_a_terminal(
            master, context, "Station A terminal NG"
        )

        context.request_station_b_skip.assert_called_once_with(
            "Station A terminal NG"
        )
        publish_cancellation.assert_called_once()


if __name__ == "__main__":
    unittest.main()

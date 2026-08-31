import unittest
from types import SimpleNamespace

from inspection_common import NodeId
from inspection_master.master_node import MasterNode


class UnexpectedReplayClient:
    """BOOT에서 replay client를 조회하면 시험을 실패시킵니다."""

    def service_is_ready(self) -> bool:
        raise AssertionError("replay service must not be queried before Log READY")


class StationReplayGuardTests(unittest.TestCase):
    def test_replay_waits_until_log_worker_is_ready(self) -> None:
        master = SimpleNamespace(
            worker_states={NodeId.LOG: SimpleNamespace(ready=False)},
            _station_replay_inflight=False,
            _station_replay_completed_generation=-1,
            _station_replay_generation=0,
            session_id="session-1",
            station_replay_client=UnexpectedReplayClient(),
        )

        MasterNode._request_station_result_replay(master)


if __name__ == "__main__":
    unittest.main()

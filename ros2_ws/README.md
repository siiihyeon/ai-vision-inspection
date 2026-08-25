# ROS 2 Jazzy Workspace

`src/basic_packages`는 공통 계약, `src/nodes`는 네 실행 노드입니다. 인터페이스 2.1.0은 Vision cancellation/replay baseline입니다.

Vision Node를 완성하기 위한 확정·미결정·실험 항목과 파라미터 위치는 [Vision Node 완성 결정표](src/nodes/inspection_vision/README_COMPLETION_CHECKLIST.md)를 기준으로 합니다.

## 검증 순서

```bash
source /opt/ros/jazzy/setup.bash
python3 tools/verify_skeleton.py
python3 tools/test_domain_contracts.py
python3 tools/test_vision_algorithms.py
colcon build --symlink-install --cmake-force-configure \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 launch inspection_bringup inspection_system.launch.py profile:=sim
```

정적 검사만 통과했다고 ROS type support 생성이나 런타임 호환이 증명되는 것은 아닙니다. Pull Request의 필수 인수 조건은 Jazzy에서 `colcon build`와 관련 package test를 통과하는 것입니다.

## 주요 endpoint

| Endpoint | 형식 | 방향 |
|---|---|---|
| `/inspection/master/heartbeat` | `MasterHeartbeat` | Master → workers |
| `/inspection/{node}/heartbeat` | `NodeHeartbeat` | workers → Master |
| `/inspection/{node}/get_status` | `GetNodeStatus` | Master → all |
| `/inspection/master/operator_command` | `OperatorCommand` Service | CLI/HMI → Master |
| `/inspection/{worker}/initialize` | `InitializeNode` Action | Master → workers |
| `/inspection/master/system_command` | `SystemCommand` | Master → workers |
| `/inspection/control/position_settled` | `PositionSettled` | Control → Master (Mega 자율 이동 결과, Action 아님) |
| `/inspection/vision/capture_product` | `CaptureProduct` Action | Master → Vision |
| `/inspection/vision/station_result` | `StationResult` | Vision → Master |
| `/inspection/master/inference_cancellation` | `InferenceCancellation` | Master → Vision |
| `/inspection/vision/inference_cancellation_ack` | `InferenceCancellationAck` | Vision → Master |
| `/inspection/log/replay_station_results` | `ReplayStationResults` Service | Master → Log |
| `/inspection/master/product_result_locked` | `ProductResultLocked` | Master → Vision/Control/Log |
| `/inspection/control/actuate_product` | `ActuateProduct` Action | Master → Control |
| `/inspection/log/event` | `LogEvent` | all → Log |
| `/inspection/log/persisted_ack` | `LogPersistedAck` | Log → producers |

QoS 기준은 Heartbeat Best Effort/last 1, 업무 이벤트 Reliable, 최신 상태 Reliable/transient-local/last 1입니다. Reliable만으로 process restart 유실이 해결되지 않으므로 중요 결과와 로그는 애플리케이션 보존·재전송을 병행합니다.

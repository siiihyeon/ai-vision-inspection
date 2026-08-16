# v2 Skeleton Release Manifest

- Release: `Vision-Inspection-v2-skeleton`
- Interface baseline: `inspection_interfaces 2.0.0`
- Python packages: `0.2.0`
- Target: Ubuntu 24.04 / ROS 2 Jazzy / Python 3.12
- Generated: 2026-08-16 (Asia/Seoul)

## 포함 범위

- 7 ROS packages, 4 executable nodes
- 20 Messages, 1 Service, 4 Actions
- common UUID/digest/idempotency/QoS/init/spool
- Master aggregation/reorder domain
- Control Action endpoints and fail-safe adapter boundary
- Vision Capture/FrameBatch/FIFO/worker boundary
- Log SQLite commit/ACK repository and approved latest/attempt/revision schema families
- Arduino Mega placeholder and folder-level decision READMEs

## 현재 검증 결과

- `python -B ros2_ws/tools/verify_skeleton.py`: PASS
- `python -B ros2_ws/tools/test_domain_contracts.py`: PASS (7 tests)
- Python source AST parse: PASS (정적 검사에 포함)
- ZIP wrapper/115 entries/exclusion/full decompression/SHA-256: PASS
- ROS 2 Jazzy `colcon build`: **이 Windows host에 ROS 2/Docker가 없어 실행하지 못함**

## 병합 전 필수 Gate

Ubuntu 24.04/Jazzy에서 아래가 모두 통과하기 전 production-ready로 표시하지 않습니다.

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
python3 tools/verify_skeleton.py
python3 tools/test_domain_contracts.py
colcon build --symlink-install
source install/setup.bash
ros2 interface show inspection_interfaces/action/CaptureProduct
ros2 launch inspection_bringup inspection_system.launch.py profile:=sim
```

실제 장비 연결 전에는 각 package README의 결정표, adapter test, hardware-in-loop test가 추가로 필요합니다.

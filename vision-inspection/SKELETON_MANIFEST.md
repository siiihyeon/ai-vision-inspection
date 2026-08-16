# v2 Skeleton Release Manifest

- Release: `vision-inspection-modifed-skeleton` (modified v2)
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
- `python -B ros2_ws/tools/test_domain_contracts.py`: PASS (10 tests)
- Python source AST parse: PASS (정적 검사에 포함)
- Runtime source의 legacy hardware-trigger/LED 계약 제거 검사: PASS
- 중첩 `.git`, `build/`, `install/`, `log/` 제외 검사: PASS
- ROS 2 Jazzy `colcon build`: **현재 Codex 실행 계정에서는 WSL/ROS 2에 접근할 수 없어 실행하지 못함**

- ZIP wrapper/117 files/전체 압축 해제/원본 대비 SHA-256: PASS

## 수정 레퍼런스 반영

- `ai-vision-inspection-final`의 최신 설계 문서와 11개 XLSX 원본을 반영했습니다.
- 과거 hardware global trigger와 조명 제어를 설명하는 PDF는 `구현_전_상세설계/legacy_reference/`로 격리하고 v2 비적용 경고를 추가했습니다.
- 현재 계약은 HIKROBOT MVS `GIGE_ACTION_COMMAND` software trigger, 외부 LED controller 상시점등, path-only FrameBatch FIFO 추론을 기준으로 합니다.

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

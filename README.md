# AI 비전검수 통합 제어 시스템

컨베이어 제품을 두 Vision Station에서 촬영·추론하고, Master가 A/B 결과를 결합해 선별하는 ROS 2 프로젝트입니다. 기준 환경은 Ubuntu 24.04, ROS 2 Jazzy, Python 3.12, Arduino Mega + TB6600입니다.

## 현재 산출물의 성격

이 저장소는 수정 레퍼런스를 반영한 **modified v2 팀 협업용 통신·공정 구현**입니다. Message/Service/Action, 상태 소유권, 멱등성, 파일경로 FIFO, SQLite commit/ACK 경계와 Master의 8개 공정 블록이 들어 있습니다. Vision에는 MVS Action Command/callback adapter, 원자 RGB 저장소, persistent Queue journal과 worker runtime이 구현되어 있습니다. 다만 Ubuntu용 MVS 실장비 검증, 실제 추론 모델, Mega serial protocol/TB6600·액추에이터 adapter는 아직 완료되지 않았습니다. 따라서 지금 상태를 생산 장비에 연결하면 안 됩니다.

남은 값은 각 폴더 README의 결정표에 따라 확정합니다. Vision은 model plugin을 주입하고 MVS/GigE/GPU hardware acceptance를 통과해야 하며, Control은 실제 장비 adapter를 구현한 뒤 전체 통합 시험으로 진행합니다.

## 확정된 핵심 Workflow

```text
Control PositionSettled
  → Master CaptureProduct 요청
  → Vision GIGE_ACTION_COMMAND broadcast
  → station 필수 camera frame 전체 수신
  → host arrival monotonic 기준 frame_arrival_skew_us 검증
  → demosaic RGB PNG atomic 저장 + SHA-256
  → FrameBatch(image file paths) bounded FIFO enqueue
  → CaptureProduct 성공
  → 공유 모델 worker 병렬 추론
  → Vision StationResult / StationInferenceFailed
  → Master A/B 결합
  → Sensor3에서 미완료 FORCED_NG 및 ProductResultLocked
  → Control 선별
  → Log SQLite commit 후 ACK
```

- 촬영 재시도는 같은 `capture_id`, 증가한 attempt로 station 필수 카메라 전체를 다시 촬영하며 최대 2 attempts입니다.
- Queue가 차면 Capture Action은 실패하지 않고 저장된 같은 FrameBatch를 보존한 채 `ENQUEUE_BLOCKED`로 유지됩니다. Master는 `PAUSED`, 공간 복구 후 enqueue 성공과 함께 재개합니다.
- Queue에는 raw frame이 아닌 `frame_batch_id`와 절대 이미지 파일 경로만 들어갑니다.
- FIFO는 worker가 꺼내는 순서까지 보장합니다. 병렬 완료 순서는 Master의 `fifo_sequence` reorder buffer가 정렬합니다.
- 제품 결과 적용 deadline은 Sensor3입니다. 명시적 station 실패는 즉시 `FORCED_NG` 후보로 기록하고, Sensor3에서만 최종 판정을 잠금합니다. Sensor3 시 미완료도 `FORCED_NG`입니다.
- RGB PNG가 canonical 파일입니다. OpenCV adapter는 로딩 직후 BGR→RGB 변환 후 모델에 전달해야 합니다.
- LED는 외부 controller로 상시점등합니다. ROS/Arduino에는 밝기나 ON/OFF 제어 계약이 없습니다.
- trigger 요청 직전·반환 직후의 host monotonic/wall 시각과 각 camera raw/domain timestamp를 보존합니다. 동기화 여부가 미정인 camera timestamp는 skew 계산에 사용하지 않습니다.
- 성공한 Capture Result는 `error_code=0`, `reason=""`입니다. 경고는 `warning_codes`가 아니라 `LogEvent`로 보냅니다.
- 시스템 상태명은 수정 레퍼런스와 동일하게 `BOOT/INITIALIZING/READY/RUN_SYS/PAUSING/PAUSED/FAULT_STOP/RESETTING`을 사용합니다.

## 폴더와 소유권

| 폴더 | 소유 책임 | 남은 결정표 |
|---|---|---|
| `inspection_master` | 시스템 FSM, 제품 ID/FIFO, A/B 결합, 최종 잠금 | 해당 패키지 README |
| `inspection_control` | Mega, 센서, TB6600, 액추에이터 | 해당 패키지 README |
| `inspection_vision` | MVS capture, RGB 파일, queue/worker, station 결과 | 해당 패키지 README |
| `inspection_log` | SQLite, ACK, projection, 보존정책 | 해당 패키지 README |
| `inspection_interfaces` | v2 노드 간 계약 | 해당 패키지와 msg/action/srv README |
| `inspection_common` | ID/digest/QoS/초기화/spool | 해당 패키지 README |
| `inspection_bringup` | sim/hardware 설정과 launch | config README |
| `firmware/arduino_mega` | Mega firmware placeholder | firmware README |
| `구현_전_상세설계` | 승인사항과 미결정사항 단일 목록 | 설계 README |

## 빌드와 검사

Ubuntu 24.04 / ROS 2 Jazzy:

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
python3 tools/verify_skeleton.py
python3 tools/test_domain_contracts.py
python3 tools/test_vision_node.py
colcon build --symlink-install
source install/setup.bash
ros2 launch inspection_bringup inspection_system.launch.py profile:=sim
```

`sim`은 deterministic fake 카메라와 wiring 전용 fake model을 자동 사용합니다. 이는 통신·저장·Queue 시험용이며 판정 성능을 의미하지 않습니다. `hardware`는 미확정 카메라/시간/model 설정이 남아 있는 동안 `INIT_BLOCKED`가 정상입니다.

## 변경 금지 원칙

- `inspection_interfaces` 2.0.0의 필드나 enum을 한 노드 담당자가 단독 변경하지 않습니다.
- `product_id`, `fifo_sequence`, A/B 결합과 최종 판정은 Master만 소유합니다.
- Vision은 이미지 파일을 소유하고 Log는 메타데이터·digest를 저장합니다. 삭제는 확정된 Log 보존정책만 수행합니다.
- 미결정 값에 임의의 생산 기본값을 넣지 않습니다. `hardware.yaml`의 빈 값/0은 의도적인 fail-closed 표시입니다.
- `build`, `install`, `log`, runtime data는 배포 ZIP에서 제외합니다.

# AI 비전검수 통합 제어 시스템

컨베이어 제품을 두 Vision Station에서 촬영·추론하고, Master가 A/B 결과를 결합해 선별하는 ROS 2 프로젝트입니다. 기준 환경은 Ubuntu 24.04, ROS 2 Jazzy, Python 3.12, Arduino Mega + TB6600입니다.

## 현재 산출물의 성격

이 저장소는 **Vision Node 2 정책을 반영한 interface 2.1 공정 구현**입니다. Ubuntu MVS 5.0.2 Action1 adapter, Mono8 촬영 계약, packet-loss 검증, station batch worker, A terminal NG의 B 취소, Sensor3 lock, Vision durable 결과/replay, Log 보고서·10,000장 보존까지 연결되어 있습니다. Mega serial protocol/TB6600·액추에이터 adapter와 모델 전처리·출력 decoder는 아직 placeholder입니다. 따라서 남은 설정과 실장비 검증 없이 생산 라인을 운전하면 안 됩니다.

Vision Node의 확정값, 미결정 정책, 실험값, 모든 파라미터 수정 위치는 [Vision Node 완성 결정표](ros2_ws/src/nodes/inspection_vision/README_COMPLETION_CHECKLIST.md)를 단일 기준으로 사용합니다. 해당 표의 구현 차단 항목을 확정하면 placeholder를 실제 장비 adapter와 추론 알고리즘으로 교체할 수 있고, 이후 실장비 인수시험을 통과해야 생산 승인이 됩니다.

기존 `README_비전검수_워크플로우.pdf`도 현재 Action1·Mono8·비동기 추론 정책에 맞춰 갱신되어 있습니다. 세부 파라미터는 PDF 요약이 아니라 위 완성 결정표와 hardware YAML을 기준으로 합니다.

## 확정된 핵심 Workflow

```text
Control PositionSettled
  → Master CaptureProduct 요청
  → Vision GIGE_ACTION_COMMAND broadcast
  → station 필수 camera frame 전체 수신
  → host arrival monotonic 기준 frame_arrival_skew_us 검증
  → 2448×2048 Mono8 PNG atomic 저장 + SHA-256 + packet_loss=0 검증
  → FrameBatch(image file paths) bounded FIFO enqueue
  → CaptureProduct 성공
  → A 3-view/B 1-view PyTorch batch worker 추론
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
- 1-channel Mono8 PNG가 canonical 파일입니다. resize/normalization은 모델 계약 주입 전까지 placeholder입니다.
- Action1은 즉시 실행(`scheduled=false`)하며 A=`key/mask 1/1`, B=`2/2`로 분리합니다. PTP 상태와 무관하게 host monotonic frame-arrival 시각만 skew 판정에 사용합니다.
- 시작 시 네 카메라의 모델·serial·IP·firmware를 SDK로 조회합니다. 기대 firmware 값이 비어 있으면 네 대의 버전이 서로 동일한지만 검증하고 자동 firmware update는 하지 않습니다.
- Station A terminal NG 또는 실패는 Station B의 미시작 촬영, 저장 후 enqueue, queued job, pre-forward, active-forward 결과를 단계별로 취소합니다. 시작된 forward 자체는 강제 종료하지 않습니다.
- Vision terminal 결과는 local spool에 먼저 기록하며 Log가 같은 session에서 재전송할 수 있습니다.
- 정상 프로그램 종료 때 제품별 CSV와 성능 summary를 만들고, 완성 이미지는 Log가 최근 10,000장만 유지합니다.
- LED는 외부 controller로 상시점등합니다. ROS/Arduino에는 밝기나 ON/OFF 제어 계약이 없습니다.
- trigger 요청 직전·반환 직후의 host monotonic/wall 시각과 각 camera raw/domain timestamp를 보존합니다. 동기화 여부가 미정인 camera timestamp는 skew 계산에 사용하지 않습니다.
- 성공한 Capture Result는 `error_code=0`, `reason=""`입니다. 경고는 `warning_codes`가 아니라 `LogEvent`로 보냅니다.
- 시스템 상태명은 수정 레퍼런스와 동일하게 `BOOT/INITIALIZING/READY/RUN_SYS/PAUSING/PAUSED/FAULT_STOP/RESETTING`을 사용합니다.

## 폴더와 소유권

| 폴더 | 소유 책임 | 남은 결정표 |
|---|---|---|
| `inspection_master` | 시스템 FSM, 제품 ID/FIFO, A/B 결합, 최종 잠금 | 해당 패키지 README |
| `inspection_control` | Mega, 센서, TB6600, 액추에이터 | 해당 패키지 README |
| `inspection_vision` | MVS capture, Mono8 파일, queue/worker, station 결과 | 패키지 README와 `README_COMPLETION_CHECKLIST.md` |
| `inspection_log` | SQLite, ACK, projection, 보존정책 | 해당 패키지 README |
| `inspection_interfaces` | v2 노드 간 계약 | 해당 패키지와 msg/action/srv README |
| `inspection_common` | ID/digest/QoS/초기화/spool | 해당 패키지 README |
| `inspection_bringup` | sim/hardware 설정과 launch | config README |
| `firmware/arduino_mega` | Mega firmware placeholder | firmware README |
| `구현_전_상세설계` | 노드 공통 승인 정책 요약 | 설계 README |

## 빌드와 검사

Ubuntu 24.04 / ROS 2 Jazzy:

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
python3 tools/verify_skeleton.py
python3 tools/test_domain_contracts.py
colcon build --symlink-install
source install/setup.bash
ros2 launch inspection_bringup inspection_system.launch.py profile:=sim
```

`sim`은 작은 Mono8 PNG를 만드는 fake capture/model 골격을 사용합니다. `hardware`는 acquisition/skew/timeout/model 계약과 실장비 검증이 끝날 때까지 `INIT_BLOCKED`가 정상입니다.

## 변경 금지 원칙

- `inspection_interfaces` 2.1.0의 필드나 enum을 한 노드 담당자가 단독 변경하지 않습니다.
- `product_id`, `fifo_sequence`, A/B 결합과 최종 판정은 Master만 소유합니다.
- Vision은 완성 이미지 파일을 저장만 하고 Log가 메타데이터·digest와 삭제 권한을 소유합니다.
- 미결정 값에 임의의 생산 기본값을 넣지 않습니다. `hardware.yaml`의 빈 값/0은 의도적인 fail-closed 표시입니다.
- `build`, `install`, `log`, runtime data는 배포 ZIP에서 제외합니다.

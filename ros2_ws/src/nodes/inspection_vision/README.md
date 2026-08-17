# inspection_vision

VisionNode의 비추론 영역 구현입니다. 카메라 초기화·software trigger·프레임 상관관계,
RGB PNG 원자 저장, path-only station FrameBatch Queue, worker 수명주기, 재시작 journal,
Master/Log 통신을 구현했습니다. 제품 최종 판정과 Station A/B 결과 결합은 Master가
소유합니다. 실제 불량 판정 수학식과 production model adapter만 의도적으로 비워
두었습니다.

> `sim`은 배선과 통신 시험용 deterministic fake 카메라/모델입니다. 판정 성능을
> 의미하지 않습니다. `hardware`는 미확정 필수값이나 model artifact가 비어 있으면
> `INIT_BLOCKED`가 되는 fail-closed 프로필입니다.

## 확정된 경계

- ROS 2 Jazzy / Ubuntu 24.04 / Python 3.12
- HIKROBOT MV-CS050-10GC, firmware `4.0.43`, Linux MVS 목표 `5.0.2`
- `MV_CC_GetSDKVersion()` raw 값은 실장비에서 vendor 대응표로 확정하며, 0인 동안
  hardware profile은 READY가 되지 않고 확정 후 다른 raw SDK는 초기화에서 거부
- LED 소프트웨어 제어 없음: 외부 LED controller 상시 점등
- 즉시 실행 `GIGE_ACTION_COMMAND`; scheduled Action 사용 안 함
- `TriggerSelector=FrameBurstStart`, burst count 1, `TriggerSource=Action1`
- DeviceKey `0x13572468`; Station A GroupKey 1, Station B GroupKey 2;
  GroupMask `0xFFFFFFFF`
- Station A 카메라 3대, Station B 카메라 1대; ID 배열 순서가 model 입력 순서
- SDK callback 진입 즉시 host monotonic/wall 시각 기록
- 프레임 연결 우선순위: `nTriggerIndex` → `nFrameNum` → 요청 이후 callback 도착
- callback에서 BayerRG8 source를 즉시 복사하고 MVS `MV_CC_ConvertPixelTypeEx`로
  RGB8 packed 변환
- canonical 파일은 RGB PNG, compression level 3; OpenCV worker는 BGR decode 후
  RGB로 변환
- 공식 `frame_arrival_skew_us`는 필수 카메라 callback host monotonic의 max-min
- skew 초과/부분 실패 시 동일 `capture_id`로 해당 station 필수 카메라 전체 재촬영
- 최대 촬영 attempt 2; retry 전에 SDK buffer clear·재무장, 고정 sleep 없음
- `CaptureProduct` 성공 시점은 모든 PNG/manifest 검증과 path-only job enqueue 완료
- 성공 Result는 `error_code=0`, `reason=""`; warning은 `LogEvent`로만 기록
- Queue 순서키는 `fifo_sequence`; `get()` 순서까지 FIFO
- Queue 초기 capacity 16, warning 75%, full 시 `ENQUEUE_BLOCKED`, 50% 이하에서 재개
- Queue full은 Action 실패가 아닙니다. 같은 process에서 저장된 동일 FrameBatch를
  보존하고 PAUSED 복구 뒤 enqueue합니다.
- Queue 총 timeout은 enqueue부터 결과 확정까지입니다. Sensor3 deadline은 Master의
  별도 정책입니다.
- 초기 worker 2개가 한 model 인스턴스를 공유하며 model lock을 활성화합니다.
- file read 1회 재시도, inference 1회 재시도, model warmup 실패는 초기화 실패
- `ProductResultLocked` 뒤 대기 job 제거, running/late 성공 억제, journal `locked`
- process 재시작 전 enqueue commit된 `pending/running`만 SQLite에서 복원
- 저장만 하고 enqueue가 끝나지 않은 배치는 재시작 후 복원·재촬영하지 않으며
  동일 capture 요청을 실패 처리해 Master가 Sensor3에서 `FORCED_NG`를 확정하게 함
- 카메라 최종 실패 뒤 0/0.5/1.0초 간격 최대 3회 재연결, station 시험 촬영 3회
- 한 카메라가 복구되지 않으면 Vision 전체 station을 `INIT_BLOCKED`로 차단
- 이미지 directory 2770, file 0660, Linux umask 0007
- disk warning/pause/critical 초기값 80/90/95%와 free 20/10/5 GiB
- 주요 telemetry는 schema v2 `LogEvent.payload_json` 하나로 묶고 ACK 전 SQLite
  producer spool에 보존

## MasterNode 통신 계약

기준 브랜치 `fix/master-node-review`의 Master와 다음 이름/identity로 연결됩니다.

| 방향 | ROS endpoint | 핵심 규칙 |
|---|---|---|
| Master → Vision | `/inspection/vision/initialize` | 같은 process에서 반복 호출 가능, resource 재생성은 active capture 중 거부 |
| Master → Vision | `/inspection/vision/capture_product` | command digest와 product/station/capture/camera set 엄격 검증 |
| Vision → Master | `/inspection/vision/queue_state` | 상태가 바뀔 때만 발행 |
| Vision → Master | `/inspection/vision/station_result` | 유한 score, revision ≥ 1, 모든 identity 포함 |
| Vision → Master | `/inspection/vision/station_inference_failed` | enqueue 이후 실패만 발행, 모든 identity 포함 |
| Master → Vision | `/inspection/master/product_result_locked` | queued/running 결과 잠금 및 late success 억제 |
| Vision ↔ Log | `/inspection/log/event`, `/inspection/log/persisted_ack` | commit ACK 전 producer spool 보존 |

촬영 전 실패는 `CaptureProduct.Result.success=false`로만 반환합니다. enqueue 이후
worker 실패는 `StationInferenceFailed`로 발행합니다. 이 둘을 동시에 보내지 않습니다.

Master Python 코드는 변경하지 않았습니다. hardware YAML의 Master camera ID만 Vision과
같은 `CAM_A_1..3`, `CAM_B_1`로 맞춘 호환성 변경이 있습니다. sim ID는 기준 브랜치와
같습니다. ROS interface IDL도 변경하지 않았습니다.

## 처리 흐름

1. `InitializeNode`가 storage, queue journal, camera backend, model warmup을 준비합니다.
2. NodeBase가 session을 적용하고 READY로 바꾼 뒤 worker를 시작합니다. 따라서 이전
   session 결과가 새 session 적용 전에 발행되지 않습니다.
3. Capture goal의 command/session/epoch/digest와 camera 순서를 확인합니다.
4. station/camera lock을 잡고 MVS buffer를 비운 뒤 immediate Action Command를 보냅니다.
5. callback metadata로 이번 요청 이후의 프레임만 연결합니다.
6. host receive skew를 계산합니다. 초과하면 실패 attempt 파일을 보존하고 전체 재촬영합니다.
7. RGB PNG와 manifest를 temp write → file fsync → atomic rename → directory fsync →
   readback/SHA/PNG 검증 순서로 저장합니다.
8. journal `pending(enqueue_committed=0)`을 먼저 기록합니다.
9. Queue insert와 `enqueue_committed=1`을 같은 in-process critical section에서 완료한
   뒤에만 Capture Action을 성공시킵니다.
10. worker는 path를 OpenCV로 읽고 공유 model adapter를 호출합니다. 완료 순서는
    비동기일 수 있으며 Master가 identity와 revision으로 결합합니다.

## 파일 구조와 책임

| 파일 | ROS 의존 | 책임 |
|---|---:|---|
| `vision_node.py` | 예 | Action/topic/service 변환, Node 수명주기, Log spool |
| `vision_runtime.py` | 아니오 | capture→journal→FIFO→worker orchestration |
| `capture_service.py` | 아니오 | 2 attempts, skew, 전체 station 재촬영 |
| `capture_contract.py` | 아니오 | raw frame, artifact, batch 불변식 |
| `camera_backend.py` | 아니오 | camera/action config와 deterministic sim backend |
| `mvs_backend.py` | 아니오 | MVS lazy import, callback, Action Command, RGB 변환, 복구 |
| `artifact_store.py` | 아니오 | RGB PNG/manifest atomic 저장과 disk pressure |
| `inference_queue.py` | 아니오 | bounded FIFO, deadline, shared model worker pool |
| `queue_journal.py` | 아니오 | SQLite pending/running/done/failed/locked 및 복원 |
| `model_adapter.py` | 아니오 | production model plugin 계약과 OpenCV RGB loader |

처음 읽는 팀원은 [코드 읽기 가이드.md](코드%20읽기%20가이드.md)를 먼저 보십시오.
실험값과 아직 결정할 모든 항목은
[실험_파라미터와_미결정사항.md](실험_파라미터와_미결정사항.md)에 있습니다.
실장비 연결 순서는 [MVS_실장비_검증절차.md](MVS_실장비_검증절차.md)에 있습니다.
현재 실행에서 통과/미검증된 항목은 [검증결과.md](검증결과.md)에 있습니다.

## model plugin 계약

`vision.model.factory`는 `package.module:factory` 형식입니다. factory는 다음 인자를
받아 `warmup()`, `infer(images_rgb)`, `close()`, `model_version`을 제공하는 객체를
반환해야 합니다.

```python
def create_model(*, model_path: Path, device: str): ...
```

`infer()` 결과는 `StationInference(verdict, score, model_version)`입니다. verdict는
PASS=1/NG=2만 허용하고 score가 NaN/Inf이면 성공 메시지를 보내지 않습니다. model
artifact는 load 전에 SHA-256을 검증합니다. 전처리 shape, normalize, ROI, threshold,
camera 결과 결합 수학식은 model 담당 구현에서 확정합니다.

## 검증 명령

ROS 없는 Python 3.12 환경:

```bash
python3 -m pyflakes ros2_ws/src ros2_ws/tools
python3 ros2_ws/tools/verify_skeleton.py
python3 ros2_ws/tools/test_domain_contracts.py
python3 ros2_ws/tools/test_vision_node.py
```

Ubuntu 24.04 + ROS 2 Jazzy:

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

MVS/GigE/RTX 5070 실장비 시험은 별도 hardware acceptance이며, 통과하기 전에는 이
브랜치를 생산 공정에 투입하면 안 됩니다.

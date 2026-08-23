# inspection_vision

Vision Node는 HIKROBOT 촬영, 완성된 Mono8 PNG 저장, station 단위 추론 queue와 PyTorch worker 골격을 소유합니다. 제품의 최종 불량 판정과 물리 FIFO는 Master 소유입니다. Ubuntu MVS 5.0.2 Action1 adapter는 구현됐지만 모델 전처리·출력 decoder와 실측 파라미터가 아직 주입 전이므로 hardware profile은 의도적으로 `INIT_BLOCKED`입니다.

## 완성 결정표

확정 정책, 아직 결정할 논리 정책, 실장비에서 정할 값, 현재 기본값과 각 ROS 파라미터의 정확한 수정 위치는 [README_COMPLETION_CHECKLIST.md](README_COMPLETION_CHECKLIST.md)에 모두 정리되어 있습니다. Vision 구현·시험 때는 이 문서를 체크리스트로 사용하고, 이 README는 구조와 workflow 요약으로 사용합니다.

## 확정 Workflow

```text
Master CaptureProduct
  → station/camera lock
  → GigE Action Command (A 3대 또는 B 1대)
  → 필수 frame 전체 수신
  → packet_loss_count == 0, skew, PNG/digest/2448×2048 검증
  → Mono8 PNG 완성 저장
  → path-only InferenceJob enqueue
  → A [3,1,H,W] 또는 B [1,1,H,W] 1회 forward
  → Vision durable spool → Log SQLite commit/ACK
  → StationResult 또는 StationInferenceFailed → Master
```

- 입력 센서 해상도는 `2448×2048`, ROI offset은 `(0,0)`, canonical 파일은 1-channel `MONO8_PNG`입니다. MVS `PixelFormat=Mono8` buffer를 저장하므로 RGB로 바꾸지 않습니다.
- Action1은 공통 `DeviceKey=1`, A `GroupKey/Mask=1/1`, B `2/2`, 즉시 실행 방식입니다. `MV_GIGE_IssueActionCommand` ACK의 IP 집합과 상태까지 검증합니다.
- frame skew는 PTP 지원/lock과 무관하게 각 `MV_CC_GetImageBuffer` 성공 직후 기록한 host `monotonic_ns`만 사용합니다. camera timestamp는 동기화되지 않은 진단 metadata로 남깁니다.
- SDK 검색 시 네 카메라의 model/serial/IP/firmware를 읽습니다. `expected_firmware_version=""`이면 네 대의 firmware가 서로 동일한지만 확인하며 Vision은 firmware를 업데이트하지 않습니다.
- 전처리 resize는 종횡비를 유지하며, normalization과 모델 입출력 decoder는 모델 계약 주입 때 구현합니다. 전처리 tensor/이미지는 파일로 보존하지 않습니다.
- Station A는 세 view를 한 batch로 끝까지 forward합니다. view 하나라도 NG이면 전체 forward 직후 terminal NG 하나를 Master에 보냅니다. 이 NG는 이후 revision으로 PASS가 될 수 없습니다.
- 파일 read만 한 번 재시도합니다. timeout과 model error는 재시도하지 않습니다. CUDA OOM은 현재 제품 실패, CPU fallback 금지, Vision 재초기화를 위한 PAUSE 대상입니다.
- 같은 `capture_id`로 station 필수 카메라 전체를 최대 두 번 촬영합니다. 촬영 재시도 사이의 인위적 대기시간은 없습니다.
- 완성 canonical 파일은 Vision이 삭제하지 않습니다. 취소 후 남은 파일도 `VISION_CAPTURE_DISCARDED`로 Log에 넘기며, Log만 10,000장 보존 정책에 따라 삭제합니다.

## A terminal NG 이후 B 취소 단계

| B 상태 | 동작 |
|---|---|
| Sensor2 전/촬영 전 | B station을 `SKIPPED`, 촬영하지 않음 |
| 파일 저장 완료, enqueue 전 | enqueue하지 않고 Log에 폐기 메타데이터와 경로 기록 |
| queue 대기 | 해당 제품·B job을 queue에서 제거 |
| worker load 후 forward 전 | forward 시작하지 않고 취소 |
| forward 시작 후 | 강제 종료하지 않고 완료 후 결과를 폐기 |
| B 결과가 이미 발행됨 | 이력은 보존하지만 A terminal NG가 최종 NG를 지배 |

Sensor3에서도 제품 전체 cancellation scope를 즉시 설치합니다. 이후 완료되는 forward 결과는 Master에 발행하지 않습니다.

## Queue, 종료, timeout

- 공식 순서키는 `fifo_sequence`; station A/B 결과 결합은 Master가 합니다.
- Queue capacity 초기값은 16 job, worker 초기값은 1, model lock은 활성화입니다. 계산의 시작식은 `ceil(최대 제품유입률 × 최악 enqueue→결과시간 × 안전계수)`이며 A/B job 발생률, worker 수, model lock 직렬화, GPU memory 한계를 함께 대입한 뒤 생산 시험으로 조정합니다.
- 프로그램 정상 종료 시 대기 job은 취소합니다. active forward는 3초 soft timeout을 기록하되 강제 종료하지 않고 끝까지 기다립니다.
- Log는 A/B별 enqueue→결과 p99.9를 계산합니다. station별 최소 10,000 표본, 안전계수 1.2, 같은 모델·config fingerprint일 때만 다음 실행 후보가 됩니다. `auto_apply=false`가 기본입니다.

## 설정 파일 위치

| 분류 | 파일 | 입력할 값 |
|---|---|---|
| 촬영 | `inspection_bringup/config/vision_capture.hardware.yaml` | MVS 경로, firmware 기대값, acquisition timeout, skew, packet delay, 카메라 세부값 |
| 모델 | `inspection_bringup/config/vision_model.hardware.yaml` | `.pt` 경로, `Model_v_1`, SHA-256, warmup, worker/model lock |
| 운영 | `inspection_bringup/config/vision_runtime.hardware.yaml` | queue capacity, A/B timeout, spool, 자동 timeout 적용 |

현재 카메라 매핑은 A=`DA9880512(.13)`, `DA9880516(.11)`, `DA7552836(.14)`, B=`DA7838410(.12)`이고 모두 `192.168.10.0/24` 대역을 전제로 합니다. `packet_size=1500`, 초기 `packet_delay_ticks=5000`, queue=16, worker=1, model lock=true, disk warning/stop=`90%/95%`, GPU sampling=200 ms, warmup=10회입니다.

Ubuntu에서는 HIKROBOT MVS 5.0.2 x86_64를 `/opt/MVS`에 설치하고 Vision 프로세스를 시작하기 전에 `MVCAM_COMMON_RUNENV=/opt/MVS/lib`와 `LD_LIBRARY_PATH=/opt/MVS/lib/64:$LD_LIBRARY_PATH`가 적용돼 있어야 합니다. 저장소는 vendor SDK 파일이나 firmware `.dav`를 복제하지 않습니다.

## 실제 장비 주입 전 남은 항목

- 모델 계약: A view 물리 순서, 실제 `.pt`와 SHA-256, 입력 shape/dtype, crop·resize·padding·normalization, 출력 score·threshold·불확실 처리
- Camera/MVS: Ubuntu runtime·firmware 자동 조회, Host NIC 고정 IP, exposure/gain, acquisition timeout, packet loss 0과 host arrival skew 생산값
- 공정 성능: 최대 제품 유입 속도, A/B p99.9, queue/worker/model-lock 조합별 GPU/VRAM·latency와 저장장치 시험

전체 항목과 입력 파일은 완성 결정표를 따릅니다. Bayer, white balance, demosaic, BGR→RGB는 Mono8 정책에서 필요한 입력값이 아닙니다. PTP와 camera timestamp unit도 현재 생산 skew 판정의 차단값이 아닙니다.

SHA-256은 `.pt` 파일 내용에서 계산하는 64자리 지문입니다. 파일명은 같아도 내용이 바뀌면 지문이 바뀌므로 한 session 동안 모델이 몰래 교체되는 것을 막습니다. warmup은 생산 시작 전 dummy batch를 여러 번 forward하여 초기 CUDA kernel/메모리 할당 지연을 제거하는 절차이며 기본 골격 값은 10회입니다.

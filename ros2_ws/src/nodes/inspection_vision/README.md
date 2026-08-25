# inspection_vision

Vision Node는 HIKROBOT 네 대의 Mono8 촬영, 원자적 PNG 저장, 전처리와
4-view PatchCore artifact 추론, durable station terminal 발행을 담당합니다.
제품의 최종 판정과 물리 FIFO는 Master가 소유합니다.

## Workflow

```text
Master CaptureProduct
  → station/camera lock
  → GigE Action1 (A 3대 또는 B 1대)
  → ACK·필수 frame·packet loss 0·host-arrival skew 검증
  → 2448×2048 Mono8 PNG를 fsync + atomic rename
  → path-only bounded FIFO enqueue
  → 카메라별 V threshold와 largest-component crop
  → black-padding resize → 3-channel 복제 → ImageNet normalize
  → 카메라별 PatchCore memory bank 추론
  → threshold 대비 normalized score와 margin으로 view 판정
  → 하나라도 NG이면 station NG, station score는 normalized score 최댓값
  → durable spool commit
  → StationResult 또는 StationInferenceFailed 발행
```

전경이 검출되지 않으면 정상 판정이 아니라 `PREPROCESSING` 추론 실패입니다.
`crop_1`과 `crop_2`는 메모리에서만 만들고 NG 또는 전처리 실패 때만
`vision.data_root/diagnostics` 아래에 저장합니다. 완성 canonical 이미지는
Vision이 삭제하지 않으며 Log Node가 보존 정책을 소유합니다.

## 카메라 계약

| View | Serial | IP | Exposure | Gain |
|---|---|---|---:|---:|
| CAM_A_1 | DA9880512 | 192.168.10.13 | 8000 µs | 0 dB |
| CAM_A_2 | DA9880516 | 192.168.10.11 | 5000 µs | 0 dB |
| CAM_A_3 | DA7552836 | 192.168.10.14 | 5000 µs | 0 dB |
| CAM_B_1 | DA7838410 | 192.168.10.12 | 10000 µs | 0 dB |

- Host NIC은 `192.168.10.10/24`입니다.
- 공통 Action DeviceKey는 1, A key/mask는 1/1, B는 2/2입니다.
- acquisition timeout은 250 ms, frame arrival skew limit은 50 ms,
  packet delay는 5000 ticks입니다.
- `ExposureAuto`, `GainAuto`, `BalanceWhiteAuto`와 gamma, saturation,
  sharpness, black-level 보정은 초기화 때 모두 OFF로 강제합니다. 지원되는
  boolean node는 read-back까지 검증하고, Mono8에서 숨겨지는 color node는
  `UNAVAILABLE_IN_MONO8_FEATURE_SET`으로 inventory에 명시합니다.
- 승인 firmware는 `V4.0.43 250414 1530132`입니다. 초기화 때 네 카메라에서
  조회한 값이 모두 이 문자열과 정확히 일치해야 합니다.

## Artifact v2 계약

운영 runtime은 `PYTORCH_PATCHCORE_ARTIFACT`입니다. 하나의 versioned bundle에
`CAM_A_1`, `CAM_A_2`, `CAM_A_3`, `CAM_B_1`의 독립 memory bank와 calibration을
넣습니다. Vision Node는 완성 artifact만 읽고 memory bank를 만들지 않습니다.

```text
artifact/
├── manifest.json
├── CAM_A_1/{model.pt,calibration.json}
├── CAM_A_2/{model.pt,calibration.json}
├── CAM_A_3/{model.pt,calibration.json}
└── CAM_B_1/{model.pt,calibration.json}
```

전체 상대경로와 파일 내용을 합산한 SHA-256이 YAML의
`vision.model.sha256`과 일치해야 합니다. View별 `v_threshold`, 판정 threshold,
normalized margin은 반드시 artifact manifest에만 존재해야 하며 ROS parameter나
코드 fallback으로 두지 않습니다. `parameters_by_view`에는 각 view의 backbone,
feature layer, coreset ratio, reweighting k, 입력 해상도, margin과 나머지 PatchCore
설정을 각각 완전한 형태로 저장합니다. View마다 설정과 memory-bank shape가 달라도
되며 전역 `parameters` 항목은 v2에서 허용하지 않습니다.

## 장애와 session 정책

- 파일 읽기 오류만 한 번 재시도합니다. 전처리, timeout, 모델 오류는
  같은 입력으로 재시도하지 않습니다.
- CUDA OOM은 현재 제품의 추론 실패입니다. CPU fallback 없이 Vision을
  `DEGRADED`로 내리고 대기 queue를 닫으며, Master의 InitializeNode 재시도에서
  모델·queue·worker를 새 객체로 재생성합니다.
- terminal 결과는 먼저 SQLite durable spool에 enqueue되어야 합니다.
  spool commit이 실패하면 terminal ROS message를 억제하고 `DEGRADED`로 전환합니다.
- 새 session 성공 시 capture idempotency/cancellation/timing cache만 비웁니다.
  아직 Log ACK를 받지 않은 spool record는 session을 넘어 보존합니다.
- A/B queue total timeout 초기값은 각각 3000/1500 ms입니다. 각 station 정상
  표본 10,000개 전에는 후보만 기록하며, 자동 적용은 기본적으로 꺼져 있습니다.

## 검증

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 tools/verify_skeleton.py
/usr/bin/python3 tools/test_domain_contracts.py
/usr/bin/python3 tools/test_vision_algorithms.py
colcon build --symlink-install --cmake-force-configure \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
colcon test --python-testing pytest --return-code-on-test-failure \
  --event-handlers console_direct+
colcon test-result --verbose
```

Ubuntu에 `/usr/local/bin/python3`가 함께 설치되어 있으면 ROS의 `em` module과
충돌할 수 있으므로 colcon에는 위처럼 `/usr/bin/python3`를 명시합니다.

구현 상태와 실장비 투입 전 남은 차단 사항은
[README_COMPLETION_CHECKLIST.md](README_COMPLETION_CHECKLIST.md)를 따릅니다.

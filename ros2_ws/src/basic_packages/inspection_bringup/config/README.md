# 실행 설정 결정표

Launch는 공통 `sim.yaml`/`hardware.yaml`과 Vision 전용 세 파일을 순서대로 읽습니다. 생산값을 한 파일에 섞지 않기 위한 구조입니다.

Vision의 전체 파라미터, 아직 key가 없는 모델·카메라 항목, 실험 인수조건은 [Vision Node 완성 결정표](../../../nodes/inspection_vision/README_COMPLETION_CHECKLIST.md)를 따릅니다.

| 파일 | 책임 |
|---|---|
| `hardware.yaml` | Master, Control, Log와 공통 heartbeat |
| `vision_capture.hardware.yaml` | 카메라/Action/PTP/packet/Mono8 저장 |
| `vision_model.hardware.yaml` | `.pt` identity, CUDA, warmup, worker/model lock |
| `vision_runtime.hardware.yaml` | queue, A/B timeout, disk, spool, 종료, 자동 tuning |

## 확정값

- interface `2.1.0`, heartbeat 500 ms/timeout 2,000 ms
- Action broadcast, 전체 station 재시도 최대 2회, 재시도 전 추가 대기 없음
- 카메라 A 3대 `DA9880512`, `DA9880516`, `DA7552836`; B 1대 `DA7838410`
- canonical `MONO8_PNG`, 전체 센서 ROI `2448×2048@(0,0)`, packet size 1500
- MVS Action1 즉시 실행: DeviceKey 1, A key/mask 1/1, B 2/2
- PTP와 무관하게 host monotonic arrival skew 사용
- queue 16, worker 1, model lock 활성화는 최초 생산시험용 초기값
- packet delay 초기값 5000 ticks, disk warning 90%/stop 95%
- shutdown queue soft timeout 3초, GPU/NVML sampling 200 ms
- 완성 이미지 최근 10,000장, timeout 최소 표본 A/B 각각 10,000, p99.9×1.2, 자동 적용 기본 false
- 모델 이름 형식 `Model_v_1`, runtime PyTorch TorchScript `.pt`, warmup 기본 10회, CPU fallback 금지

## Hardware에서 아직 0/빈 값으로 남겨야 하는 항목

- acquisition timeout, frame skew limit, exposure/gain
- 자동 조회 후 필요하면 입력할 승인 firmware 버전(`expected_firmware_version`)
- A/B enqueue→결과 timeout. 시험 후 보고서 p99.9를 이용합니다.
- 실제 `.pt` SHA-256, 모델 입출력/전처리 계약, queue/worker/model-lock 조정 결과
- Mega port/baud/firmware protocol, 센서·TB6600·actuator 설정과 위치 timeout

미확정 값 때문에 hardware가 `INIT_BLOCKED`되는 것은 정상입니다. 값을 임의로 채워 READY를 우회하지 마십시오.

Exposure, gain, model device, input shape, crop/resize/normalization, output threshold는 아직 ROS parameter key가 없습니다. 값이 결정되면 `vision_node.py`에 선언·검증을 추가하고 capture 또는 model hardware YAML에 넣습니다. 설치 후 `install/` 아래 복사본을 직접 수정하지 않습니다.

## Linux 경로 준비

```bash
sudo mkdir -p /var/lib/inspection/{images,log,spool,reports,config}
sudo mkdir -p /opt/inspection/models
sudo chown -R "$USER":"$USER" /var/lib/inspection /opt/inspection/models
```

모델은 `/opt/inspection/models/Model_v_1.pt`에 놓고 `sha256sum` 결과를 `vision_model.hardware.yaml`에 입력합니다. 실제 서비스 계정을 만들면 위 소유자를 그 계정으로 바꿉니다.

# 실행 설정 결정표

최종 PatchCore v3 bundle의 /opt/inspection/models 배치, 통합 SHA 계산, model YAML
갱신, launch/init 검증과 rollback은
[Artifact 운영 배포 매뉴얼](../../../nodes/inspection_vision/ARTIFACT_DEPLOYMENT_GUIDE.md)을
따릅니다.

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
- 모델 runtime `PYTORCH_PATCHCORE_ARTIFACT`, 4-view v3 공간 calibration bundle,
  warmup 기본 10회, artifact v2/CPU fallback 금지

## Hardware에서 아직 시험이 필요한 항목

- queue/worker/model-lock의 생산 조정 결과
- Mega firmware protocol, 센서·TB6600·actuator 설정과 위치 timeout

미확정 장비 계약이나 인수시험 실패 때문에 hardware가 `INIT_BLOCKED`되는 것은
정상입니다. 값을 임의로 채워 READY를 우회하지 마십시오.

Exposure/gain은 camera map에 있으며 모든 보정 OFF는 adapter가 강제합니다. Full-frame
전처리 버전, V threshold, 위치 정규화/aggregation과 판정 threshold는 ROS key로
만들지 않고 artifact v3 안에서만 관리합니다. Margin은 사용하지 않습니다. 설치 후
`install/` 아래 복사본을 직접 수정하지 않습니다.

## Linux 경로 준비

```bash
sudo mkdir -p /var/lib/inspection/{images,log,spool,reports,config}
sudo mkdir -p /opt/inspection/models
sudo chown -R "$USER":"$USER" /var/lib/inspection /opt/inspection/models
```

검증을 통과한 v3 모델 bundle을 `/opt/inspection/models` 아래에 놓고 artifact 이름과
통합 directory SHA를 `vision_model.hardware.yaml`에 입력합니다. 기존 v2 bundle은
runtime이 거부합니다. 실제 서비스 계정을 만들면 위 소유자를 그 계정으로 바꿉니다.

현재 통합 호스트에서는 Mega 포트 `/dev/ttyACM0`, baud `115200`, 카메라
firmware `V4.0.43 250414 1530132`가 hardware profile에 고정되어 있습니다.

# Vision Node 2 정책 기준

이 문서는 `feature/vision-node-2` 구현의 승인 정책 요약입니다. 과거 XLSX나 질의 기록에 RGB/Bayer, inference retry, confusion matrix가 남아 있으면 현재 코드와 이 문서가 우선합니다.

Vision Node의 모든 미결정값과 수정 위치는 [Vision Node 완성 결정표](../ros2_ws/src/nodes/inspection_vision/README_COMPLETION_CHECKLIST.md)가 단일 기준입니다. 이 문서는 노드 간 책임과 승인 정책만 요약하며 파라미터 목록을 중복 관리하지 않습니다.

## 책임과 판정

- Vision은 A 세 장과 B 한 장을 각각 station job으로 추론합니다. 제품 최종 판정은 Master가 소유합니다.
- A는 세 장을 `[3,1,H,W]` 한 batch로 한 번 forward합니다. B는 `[1,1,H,W]`입니다.
- A 첫 NG는 terminal이며 PASS로 수정할 수 없습니다. 즉시 Master에 전송되고 B는 단계별 취소됩니다.
- B active forward는 강제 종료하지 않습니다. 완료 결과만 버립니다.
- Sensor3 통과 순간 결과가 없으면 Master가 FORCED_NG로 잠급니다. 이후 Vision 결과는 공정에 적용하지 않습니다.
- Vision 종료 순간 station의 `PositionSettled`가 이미 확인된 제품만 잔류로 고정해 재촬영합니다. 당시 모터가 감속/이동 중이었다면 이후 정지하더라도 비잔류로 보고 재촬영하지 않고 station 실패 처리합니다.
- timeout/file read/model/CUDA OOM 등 모든 terminal error는 해당 제품 NG 근거입니다. file read만 1회 재시도하고 model/timeout은 재시도하지 않습니다.

## 촬영과 파일

- Camera: Station A `DA9880512`, `DA9880516`, `DA7552836`; Station B `DA7838410`.
- IP: `.13`, `.11`, `.14`, `.12` in `192.168.10.0/24`.
- 입력 전체 ROI `2448×2048@(0,0)`, raw/store `Mono8`, canonical 1-channel PNG. RGB 변환은 하지 않습니다.
- Action1은 DeviceKey 1, A key/mask 1/1, B 2/2의 즉시 command이며 PTP 상태와 무관하게 host monotonic arrival skew를 사용합니다.
- packet loss는 frame별 미복구 count가 반드시 0이어야 합니다. 재전송 count는 별도 telemetry로 보존합니다.
- 같은 capture ID, station 전체를 최대 2 attempts. 완성 파일은 Vision이 삭제하지 않고 Log만 최근 10,000장 정책으로 삭제합니다.
- disk 90% warning, 95% 신규 촬영 중지와 PAUSE.

## 모델과 GPU

- A/B 동일 PyTorch TorchScript `.pt`, 이름 `Model_v_1`, 한 session 동안 version/SHA-256 고정.
- 실제 resize/normalization/input-output decoder는 추후 주입합니다. 전처리 파일은 보존하지 않습니다.
- CUDA 필수, CPU fallback 금지. CUDA OOM은 현재 제품 실패 후 PAUSE/재초기화입니다.
- RTX 5070 Laptop GPU 8,151 MiB 기준으로 batch 3의 실제 VRAM/latency를 시험합니다. 현재 골격이 안전 용량을 보장하는 것은 아닙니다.
- warmup 기본 10회, NVML GPU/VRAM sample 200 ms입니다. 최초값은 queue 16, model lock 활성화, worker 1이며 실험 후 조정합니다.

## 종료, replay, 보고서

- 프로그램 종료가 운전 종료입니다. 정상 종료는 waiting job 취소, active forward 3초 soft timeout 후에도 완료까지 대기합니다.
- 비정상 종료 session은 CSV/보고서를 복구 생성하지 않고 timeout tuning에서 제외합니다. 원본 SQLite audit는 유지합니다.
- Vision terminal은 local spool 선기록 후 ROS publish합니다. Log commit/ACK 전에는 spool에서 지우지 않습니다.
- 같은 Master session의 Vision/Log restart만 replay합니다. Master restart 이전 session은 공정 판정에 재사용하지 않습니다.
- 정상 종료 CSV에는 제품 순서·A/B 결과·오류·forward·enqueue→결과·제품 전체 시간을 기록합니다. Summary에는 GPU/VRAM과 timeout 통계를 기록합니다. 정답 label 기반 confusion matrix/accuracy/precision/recall/F1은 만들지 않습니다.

## Timeout과 queue

- A/B enqueue→결과 p99.9를 분리 계산합니다. 최소 표본은 station별 10,000, 후보는 p99.9×1.2입니다.
- 자동 적용은 기본 false. true여도 같은 model version/SHA-256/config fingerprint일 때만 다음 정상 실행에 적용합니다.
- Queue 시작식: `capacity >= ceil(λjob × T최악 × 안전계수)`. `λjob`은 최대 제품 유입률에 제품당 실제 발생 job 수(A=1, A PASS일 때 B=1)를 반영합니다. model lock이 켜졌으면 worker 수가 많아도 forward 처리율이 직렬이라는 점과 GPU memory 한계를 함께 검증합니다.
- Capture timeout 후보는 `평균 acquisition × 2 + save overhead + validation`을 기록합니다.

## 실제 장비 전에 남은 결정/시험

- Camera/MVS, 모델, queue, 파일의 전체 입력은 Vision Node 완성 결정표를 따릅니다.
- 핵심 차단 항목은 firmware/runtime 실측, Host NIC 설정, exposure/gain/acquisition/skew, 실제 모델 입출력·전처리·threshold, 최대 제품 유입속도와 A/B timeout입니다.
- Mega serial 방식과 firmware protocol은 Vision 내부 알고리즘에는 필요 없지만 전체 라인 Sensor3/재시작 잔류 판정 통합에는 반드시 필요

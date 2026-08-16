# 상세설계 현재 상태

이 폴더의 표와 문서는 구현 근거입니다. 다만 `06_코드_골격_완성을_위한_필수_결정사항.md`는 질의 과정의 상세 기록이며, 서로 다른 시점의 미결정 표기가 남아 있을 수 있습니다. **현재 코드 baseline과 아래 승인사항이 우선**하고, 충돌 시 Pull Request에서 설계 문서까지 함께 정정합니다.

## 승인되어 v2 골격에 반영된 사항

- HIKROBOT MVS `GIGE_ACTION_COMMAND` broadcast, LED software control 없음
- Capture 성공점: 필수 frame의 RGB PNG 저장·검증 및 FrameBatch 경로 enqueue 완료
- `frame_arrival_skew_us`: host receive monotonic timestamp의 max-min
- camera raw timestamp/domain과 host wall/monotonic timestamp 모두 보존
- same `capture_id`, station 전체 재촬영, 최대 2 attempts
- Vision restart 후 제품이 station에 있으면 재촬영, 떠났으면 station 실패→FORCED_NG
- Queue full: 저장 FrameBatch 보존, Action `ENQUEUE_BLOCKED`, Master PAUSED, 복구 후 같은 batch enqueue
- station-level `InferenceJob`, path-only FIFO, `fifo_sequence`, shared model, worker 병렬
- 파일 read 1회 재시도, inference 1회 재시도
- 추론 timeout 경과는 enqueue부터 결과 확정까지 Queue 총시간으로 측정; 제품 결과 적용 deadline은 Sensor3
- A/B 결합과 최종 잠금은 Master, 늦은 결과는 진단 보존 후 적용 금지
- `warning_codes` 없음; warning은 `LogEvent`
- UUIDv4 session/command/node instance, SHA-256 digest, epoch invalidation, 500/2000 ms Heartbeat
- SQLite commit 후 Log ACK, producer local SQLite spool, approved v2 table families/QoS

## 최종 구현 전에 반드시 받을 결정 묶음

| 묶음 | 필요한 정확한 값 |
|---|---|
| 제품·물리 FSM | Sensor1/2/3 명칭·polarity·제품 매핑, `product_id` 생성/복구, `fifo_sequence` 영속화, station/actuator 위치와 conveyor 매핑 |
| Control 전장 | Mega port/baud, packet schema·CRC·ACK·retry/watchdog, pin map, sensor debounce/rearm/stuck, TB6600 calibration·속도·가감속, actuator timing/feedback |
| Camera | station별 camera serial/role, MVS SDK/firmware, Action key/group/mask, Action1/PTP 지원 시험, acquisition timeout, Bayer/pixel-format 설정 |
| Timestamp/skew | camera timestamp domain/PTP 사용 여부, host arrival 기록 지점, `frame_arrival_skew_us` 허용치 |
| Vision 파일 | `data_root`, 폴더/파일 naming, fsync/rename 규칙, disk threshold, late/failed attempt 보존 기간 |
| Queue/worker | production queue capacity, worker 수, Queue 총 timeout ms, model thread safety 확인 후 lock 유지 여부 |
| 모델 | 파일/버전/digest, 입력 크기·정규화·BGR→RGB 위치, 수학식, camera 결과→station 결과 결합, threshold/calibration, score schema |
| Master 정책 | 초기화 순서·retry/전체 상태 전이, 각 timeout/복구 횟수, Sensor3 매핑, actuator 명령 시점과 late result 진단 정책 |
| Log 운영 | DB/spool 경로, 용량 경고·정지 기준, 보존/삭제 기간, projection mapping/query, backup/복구 |
| 복구 시험 | camera station 시험촬영 판정, Mega reconnect, node restart별 제품 위치 판별 방법과 운영자 승인 흐름 |

세부 키는 각 ROS package README에 중복 없이 배정되어 있습니다. 모든 생산값은 `hardware.yaml`에 반영하고 config digest를 다시 생성해야 합니다.

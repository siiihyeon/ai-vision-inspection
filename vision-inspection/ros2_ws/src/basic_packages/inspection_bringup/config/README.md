# 실행 설정 결정표

## 확정값

- Heartbeat period 500 ms, timeout 2,000 ms
- interface 2.0.0
- trigger mode `GIGE_ACTION_COMMAND`
- capture max attempts 2
- sim worker count 1
- LED 관련 key 없음

## hardware.yaml에 반드시 채울 값

| 소유자 | 키 | 필요한 정보 |
|---|---|---|
| Control | `control.mega.*` | device path, baud와 serial protocol version |
| Control | `control.tb6600.*_config` | step calibration, 속도/가감속/방향/limit의 versioned config 경로 |
| Control | `control.sensor_config` | pin, polarity, debounce, rearm, stuck 기준 config 경로 |
| Control | `control.actuator_config` | pin, 안전상태, 동작/복귀 timing, feedback config 경로 |
| Vision | `vision.camera_ids.*` | 각 station 필수 camera serial과 role |
| Vision | `vision.gige_action.*` | MVS 시험으로 확정한 device/group key와 mask |
| Vision | `vision.frame_arrival_skew_limit_us` | 허용 host receive skew |
| Vision | `vision.queue.capacity` | production bounded depth |
| Vision | `vision.worker_count` | GPU/CPU 부하 시험 결과 |
| Vision | `vision.inference_queue_total_timeout_ms` | enqueue→결과 총 허용시간 |
| Vision | `vision.data_root` | 동일 Ubuntu PC의 canonical image root |
| Vision | `vision.model.path` | version/digest로 고정한 model artifact |
| Log | `log.database_path` | SQLite 절대 경로 |
| Log | `log.producer_spool_root` | 노드별 spool root |
| Log | `log.data_root` | Vision과 합의한 공유 root |
| Log | `log.retention_policy` | versioned 보존/삭제 정책 이름 |

값을 채운 뒤 파일 전체의 canonical SHA-256을 config digest로 배포하고, 해당 값으로 hardware initialization test를 통과시킵니다.

# 실행 설정 결정표

## 확정값

- Heartbeat period 500 ms, timeout 2,000 ms
- Initialize Action 1회 timeout 10,000 ms, 재시도 간격 1,000 ms
- 초기화 실패 3회부터 작업자 경고(최대 횟수나 FAULT_STOP 기준이 아님)
- interface 2.0.0
- trigger mode `GIGE_ACTION_COMMAND`
- capture max attempts 2
- Station A 카메라 3대, Station B 카메라 1대
- Master 활성 FIFO 개발 기본값 soft 18 / hard 20
- 액추에이터 완료 후 Master 제품 Context 보존 10분
- sim/hardware 초기 worker count 2, 공유 model lock 활성
- Action DeviceKey `0x13572468`, A/B GroupKey 1/2, GroupMask `0xFFFFFFFF`
- SDK buffer 8, RGB PNG compression level 3
- Queue 초기 capacity 16, warning 75%, resume 50%
- disk 초기 경계 used 80/90/95%, free 20/10/5 GiB
- LED 관련 key 없음

## hardware.yaml에 반드시 채울 값

| 소유자 | 키 | 필요한 정보 |
|---|---|---|
| Master | `master.sensor_ids.*` | Mega/Control 논리 센서 ID와 실제 배선·polarity 대응 |
| Master | `master.hardware_mapping_confirmed` | 센서·station·conveyor·actuator 매핑 검증 완료 여부 |
| Master | `master.station_*.position_offset_steps` | Sensor1/2 감지점부터 촬영 위치까지 보정 step |
| Master | `master.position_tolerance_steps` | open-loop 위치 오차 허용 범위 |
| Master | `master.camera_ids.*` | hardware는 `CAM_A_1..3`/`CAM_B_1`; Vision 설정과 순서까지 일치 |
| Master | `master.action.*_timeout_ms` | 위치·촬영·액추에이터·개별 컨베이어 재가동 확인의 최악 처리시간 |
| Master | `master.*stop_timeout_ms` | PAUSE·종료 안전 정지 확인 제한시간 |
| Master | `master.log_spool_*` | producer SQLite spool 경로와 경고·정지 용량 |
| Master | `master.completed_context_retention_ms` | 기본 600000 ms; 생산 주기·late result 실측 후 재검토 |
| Control | `control.mega.*` | device path, baud와 serial protocol version |
| Control | `control.tb6600.*_config` | step calibration, 속도/가감속/방향/limit의 versioned config 경로 |
| Control | `control.sensor_config` | pin, polarity, debounce, rearm, stuck 기준 config 경로 |
| Control | `control.actuator_config` | pin, 안전상태, 동작/복귀 timing, feedback config 경로 |
| Vision | `vision.camera_ids.*` | 각 station의 논리 ID와 model 입력 순서 |
| Vision | `vision.camera_serials.*` | CAM_A_2/A_3/B_1 실제 serial; A_1은 `DA9880516` |
| Vision | `vision.camera_macs.*` | A_2/A_3/B_1 실제 MAC; A_1은 `34:BD:20:8A:22:8A` |
| Vision | `vision.mvs.python_module_path` | Ubuntu MVS 5.0.2 Python wrapper 절대경로 |
| Vision | `vision.mvs.expected_sdk_version_raw` | 5.0.2와 대응함을 vendor 자료로 확인한 `MV_CC_GetSDKVersion()` 정수 |
| Vision | `vision.camera.timestamp_tick_hz` | device raw timestamp 단위; 미정이면 0/`synchronized=false` |
| Vision | `vision.camera.ptp_stability_ms` | PTP stable 판정 유지시간 실측값; 0이면 capture는 허용하되 synchronized=false |
| Vision | `vision.frame_arrival_skew_limit_us` | 허용 host receive skew |
| Vision | `vision.camera.frame_timeout_ms` | 현재 2000 ms 초기값; exposure/network 최악값으로 검증 |
| Vision | `vision.queue.capacity` | 초기 16; production bounded depth 부하시험 |
| Vision | `vision.worker_count` | 초기 2; RTX 5070 GPU/CPU 부하 시험 결과 |
| Vision | `vision.inference_queue_total_timeout_ms` | enqueue→결과 총 허용시간 |
| Vision | `vision.data_root` | 현재 `/var/lib/inspection/data`; 실제 mount/fsync/권한 검증 |
| Vision | `vision.queue_journal_path` | SQLite journal 절대경로와 quota |
| Vision | `vision.model.*` | model 절대경로, SHA-256, `module:factory`, CUDA device |
| Vision | `vision.storage.*` | 초기 disk 경계를 공정 정지 여유시간으로 검증 |
| Log | `log.database_path` | SQLite 절대 경로 |
| Log | `log.producer_spool_root` | 노드별 spool root |
| Log | `log.data_root` | Vision과 합의한 공유 root |
| Log | `log.retention_policy` | versioned 보존/삭제 정책 이름 |

현재 Master는 profile·설정/인터페이스 버전·FIFO 한도·센서 ID·카메라 ID를
canonical SHA-256 session config digest에 포함합니다. 배포 파이프라인에서는
최종 hardware.yaml 전체 snapshot digest와의 일치 검증을 추가해야 합니다.

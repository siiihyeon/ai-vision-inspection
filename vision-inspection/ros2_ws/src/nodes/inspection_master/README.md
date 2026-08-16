# inspection_master

Master는 시스템 상태, `product_id`, `fifo_sequence`, 물리 제품 추적, A/B 결과 결합과 최종 잠금의 유일한 소유자입니다.

## 골격에 구현된 경계

- worker Heartbeat/interface/process restart 감시, restart 시 epoch 증가·PAUSE
- Control/Vision/Log Initialize clients
- Position/Capture/Actuate Action clients
- `ProductLedger`, station revision, A+B 결합
- 명시적 station 실패 즉시 FORCED_NG
- Sensor3 미완료 FORCED_NG용 `lock_product_at_sensor3()` 확장점
- 병렬 완료를 `fifo_sequence`로 내보내는 `ProductResultReorderBuffer`
- Vision `ENQUEUE_BLOCKED` PAUSE와 queue recovery RESUME

## 반드시 결정 후 구현할 사항

| 항목 | 정확히 필요한 정보 |
|---|---|
| 제품 생성 | Sensor1 event와 product 생성 조건, ID 형식, 재시작 시 counter/UUID 복구 |
| FIFO | `fifo_sequence` 영속화 위치, 재시작 후 첫 sequence, 물리 이탈/중복 센서 처리 |
| 물리 매핑 | Sensor1/2/3와 station A/B, 상·하 컨베이어, actuator의 순서·거리·예상 제품 매핑 |
| 전체 FSM | BOOT/INITIALIZING/READY/RUNNING/PAUSED/RECOVERING/FAULT 전이와 명령 허용표 |
| 초기화 | worker 순서, 각 timeout/retry, 일부 실패 시 rollback, 운영자 승인 |
| Sensor3 | 어떤 대기 제품에 event를 배정하는지, bounce/중복/예상 없음 처리 |
| Actuation | PASS/NG/FORCED_NG별 명령, deadline, 통과확인과 실패 복구 |
| restart | Vision 저장 batch 존재/제품 station 잔류 판별, Control/Log restart별 epoch·복구 sequence |
| 로그 | Master가 반드시 spool해야 하는 event type과 revision 규칙 |

## 금지

- Vision 결과가 Master를 거치지 않고 actuator를 직접 움직이게 하지 않습니다.
- Sensor3 이후 late result로 `ProductResultLocked`를 수정하지 않습니다.
- `_handle_sensor_event()` TODO에 단순히 “가장 오래된 제품” 같은 추정 규칙을 넣지 않습니다.

## 인수 시험

정상 2 station PASS, station NG, 명시적 실패, Sensor3 미완료, out-of-order result, duplicate/revision, worker restart, queue full/recovery를 모두 deterministic test로 추가해야 합니다.

# Message 계약 메모

- `CommonHeader.stamp`: UTC wall clock 기록용. timeout 계산에 사용하지 않습니다.
- `NodeHealth`, `SystemState`, `StationId`, `ConveyorId`: wire/DB에서 공유하는 숫자 enum catalog입니다.
- `CommandHeader.command_epoch`: RESET 또는 restart invalidation 때 Master가 증가시킵니다.
- `PositionSettled.position_source`: 현재 기본은 `OPEN_LOOP_ESTIMATE`; 실제 encoder 사용 시 명시합니다.
- `VisionQueueState.ENQUEUE_BLOCKED`: 저장 실패가 아니라 bounded queue 포화 상태입니다.
- `StationResult.result_revision`: 같은 station 결과의 append-only revision입니다.
- `ProductResultLocked`: 이후 Vision 결과는 적용하지 않고 진단 로그만 남깁니다.
- `LogPersistedAck.acked_log_ids`와 `acked_revisions`는 같은 index의 identity pair입니다.

결정 필요 값: Sensor ID 체계, station/conveyor 실제 매핑, model score 의미·범위, camera timestamp domain 문자열 표준.

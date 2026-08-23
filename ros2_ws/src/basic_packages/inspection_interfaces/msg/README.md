# Message 계약 메모

- `CommonHeader.stamp`: UTC wall clock 기록용. timeout 계산에 사용하지 않습니다.
- `NodeHealth`, `SystemState`, `StationId`, `ConveyorId`: wire/DB에서 공유하는 숫자 enum catalog입니다.
- `CommandHeader.command_epoch`: RESET 또는 restart invalidation 때 Master가 증가시킵니다.
- `SystemCommand.target_conveyor_id`: `0(ALL_CONVEYORS)`은 전체 PAUSE/RESUME/RESET,
  `UPPER`/`LOWER`는 Station 촬영 후 해당 층만 재가동하는 명령입니다.
  특정 컨베이어 명령은 전체 `SystemState`를 바꾸지 않습니다.
- `PositionSettled.position_source`: 현재 기본은 `OPEN_LOOP_ESTIMATE`; 이때 `position_verified=false`입니다. encoder 등 독립 확인이 구현된 경우에만 true로 보고합니다.
- `ImageReference`: camera raw/domain/정규화 ns와 동기화 여부, host arrival monotonic/wall 시각을 모두 보존합니다. skew는 host monotonic 필드만 사용합니다.
- `VisionQueueState.ENQUEUE_BLOCKED`: 저장 실패가 아니라 bounded queue 포화 상태입니다.
- `StationResult.result_revision`: 같은 station 결과의 append-only revision입니다.
- `StationResult.score`: 유한한 `float32`만 허용합니다. 계산 불가 또는 `NaN`·`Inf`는
  `StationInferenceFailed`로 보고하며, 잘못 전송된 비유한 score는 Master가
  비전 결과 계약 위반과 `FORCED_NG`로 처리합니다.
- `ProductResultLocked`: 이후 Vision 결과는 적용하지 않고 진단 로그만 남깁니다.
- `InferenceCancellation`: station_id=0은 제품 전체, 1/2는 station scope입니다. Ack의 `active_result_will_be_discarded=true`는 이미 시작된 forward를 죽이지 않는다는 뜻입니다.
- `LogPersistedAck.acked_log_ids`와 `acked_revisions`는 같은 index의 identity pair입니다.

결정 필요 값: Sensor ID 체계, station/conveyor 실제 매핑, 유한값 범위 안에서의
model score 의미·정상 범위, camera timestamp domain 문자열 표준.

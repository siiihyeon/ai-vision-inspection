# Message 계약 메모

- `CommonHeader.stamp`: UTC wall clock 기록용. timeout 계산에 사용하지 않습니다.
- `NodeHealth`, `SystemState`, `StationId`, `ConveyorId`: wire/DB에서 공유하는 숫자 enum catalog입니다.
- `CommandHeader.command_epoch`: RESET 또는 restart invalidation 때 Master가 증가시킵니다.
- `SystemCommand.target_conveyor_id`: `0(ALL_CONVEYORS)`은 전체 PAUSE/RESUME/RESET,
  `UPPER`/`LOWER`는 Station 촬영 후 해당 층만 재가동하는 명령입니다.
  특정 컨베이어 명령은 전체 `SystemState`를 바꾸지 않습니다.
- `PositionSettled`: Mega가 센서 감지 후 자율로 이동·정지하고 Control이 그대로 옮겨 발행합니다. Master가 명령을 보내지 않으므로 `product_id`·`station_id`·`position_command_id`가 없고, `conveyor_id`로 어느 station cycle인지 매칭합니다(한 station엔 항상 최대 하나의 cycle만 활성 상태이므로 매칭에 모호함이 없습니다). `position_source`는 현재 기본이 `OPEN_LOOP_ESTIMATE`; 이때 `position_verified=false`입니다. encoder 등 독립 확인이 구현된 경우에만 true로 보고합니다.
- `EquipmentState`: Control이 상태가 바뀔 때만 발행하는 안전 guard mirror입니다. 주기 발행이 아니므로 구독자는 최신값을 이벤트 기반으로만 갱신합니다. `actuator_area_clear`·`estop_asserted`는 보고할 센서가 없어 이 메시지에 포함하지 않습니다.
- `ImageReference`: camera raw/domain/정규화 ns와 동기화 여부, host arrival monotonic/wall 시각을 모두 보존합니다. skew는 host monotonic 필드만 사용합니다.
- `VisionQueueState.ENQUEUE_BLOCKED`: 저장 실패가 아니라 bounded queue 포화 상태입니다.
- `StationResult.result_revision`: 같은 station 결과의 append-only revision입니다.
- `StationResult.score`: 유한한 `float32`만 허용합니다. 계산 불가 또는 `NaN`·`Inf`는
  `StationInferenceFailed`로 보고하며, 잘못 전송된 비유한 score는 Master가
  비전 결과 계약 위반과 `FORCED_NG`로 처리합니다.
- `ProductResultLocked`: 이후 Vision 결과는 적용하지 않고 진단 로그만 남깁니다.
- `LogPersistedAck.acked_log_ids`와 `acked_revisions`는 같은 index의 identity pair입니다.

결정 필요 값: Sensor ID 체계, station/conveyor 실제 매핑, 유한값 범위 안에서의
model score 의미·정상 범위, camera timestamp domain 문자열 표준.

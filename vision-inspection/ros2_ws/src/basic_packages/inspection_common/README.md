# inspection_common

노드에 공통인 기술 계약만 제공합니다.

- 숫자 enum과 노드 ID
- UUIDv4 생성/검증
- canonical JSON과 SHA-256
- bounded `IdempotencyStore`
- Heartbeat/event/state QoS
- Heartbeat watchdog, typed status, 직렬 `InitializeNode`
- producer용 `DurableLogSpool`

노드 파일에서는 ROS 기반 항목을 `inspection_common.node_base`에서 명시적으로 import합니다. package root는 ROS 없는 정적 test에서도 import할 수 있는 순수 도구만 노출합니다.

## 구현 규칙

- 노드 전용 FSM, camera, serial, model, Log projection은 이 패키지에 넣지 않습니다.
- 중요한 LogEvent는 publish 전에 local spool에 넣고 `(log_id, revision)` ACK 후 삭제합니다.
- Heartbeat timeout은 wall timestamp가 아니라 수신 host monotonic 시간으로 계산합니다.
- unknown profile은 sim으로 fallback하지 않고 즉시 실패합니다.
- hardware adapter 미구현/필수 파라미터 누락은 `INIT_BLOCKED`입니다.

## 후속 작업

- [ ] 각 producer에 `DurableLogSpool` publish/ACK loop 연결
- [ ] config 파일 자체의 canonical digest 생성·배포 도구 연결
- [ ] system reset 때 epoch별 idempotency retention/정리 기준 확정
- [ ] 장시간 운전 기준 spool capacity와 disk alarm 확정

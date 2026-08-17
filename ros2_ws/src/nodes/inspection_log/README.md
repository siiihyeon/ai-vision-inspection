# inspection_log

Log는 이벤트 영구 저장과 조회 projection을 소유합니다. 이미지 파일은 Vision 소유이며 Log는 절대 경로, digest와 metadata를 기록합니다.

## 골격에 구현된 경계

- WAL SQLite `LogRepository`
- `(log_id, revision)` 멱등 insert와 digest conflict 차단
- JSON/digest 검증
- transaction commit 이후에만 `LogPersistedAck` 발행
- products/inference jobs latest, FrameBatch attempts, camera/station revisions, faults, pending projection table family
- producer용 `inspection_common.DurableLogSpool`

## 반드시 결정할 운영값

| 항목 | 정확히 필요한 정보 |
|---|---|
| 경로 | DB 절대 경로, node별 spool root, shared data_root, filesystem/mount |
| 용량 | free-space warning/PAUSED/FAULT_STOP 임계치, DB/spool/image별 quota |
| 보존 | PASS/NG/failure/late image 및 event table별 기간, 삭제 주기, archive 방식 |
| projection | 각 `event_type` payload schema→products/captures/jobs/results/faults mapping |
| 조회 | 필요한 검색 조건, pagination/order, performance/인덱스 목표 |
| spool | 노드별 max size, resend interval/batch, ACK timeout, 오래된 session 처리 |
| backup | SQLite online backup 주기, 보관 위치, 복구/RPO/RTO 시험 |
| FK pending | dependency 대기시간, retry 횟수, 끝내 미해결일 때 경고/격리 |

## 구현 인수 조건

- ACK 전에 process kill하면 producer spool에 event가 남아 재전송됩니다.
- 같은 identity/같은 digest는 멱등 ACK, 다른 digest는 conflict fault입니다.
- projection 실패가 원본 `log_events` commit을 손상시키지 않습니다.
- 보존 삭제는 Vision 파일 소유권 handshake 없이 경로를 직접 지우지 않습니다.
- schema migration/backup/복구 test를 추가합니다.
- `InitializeNode`는 RESET·통신 복구 뒤 같은 프로세스에서 재호출될 수 있습니다.
  LogNode는 새 SQLite repository를 먼저 연 뒤 교체하고, 교체가 성공한 경우에만
  이전 repository를 명시적으로 닫습니다. 새 연결 생성 실패 시 기존 연결은
  유지하며 초기화 실패를 반환합니다.

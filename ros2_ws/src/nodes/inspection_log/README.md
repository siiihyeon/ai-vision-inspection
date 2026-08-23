# inspection_log

Log Node는 원본 이벤트의 SQLite 영구 저장, producer ACK, Vision 결과 replay, 정상 종료 보고서와 완성 이미지 삭제를 소유합니다.

## 구현된 정책

- WAL SQLite에 `(log_id, revision)` 멱등 저장 후에만 `LogPersistedAck`를 발행합니다. 같은 identity의 다른 digest는 거부합니다.
- Vision은 `StationResult/StationInferenceFailed`를 ROS로 발행하기 전에 local SQLite spool에 terminal event를 기록합니다. Log가 commit하면 ACK를 보내고 Vision spool에서 제거됩니다.
- Vision 또는 Log가 같은 Master session에서 재시작하면 Master는 `ReplayStationResults`를 page 단위로 호출합니다. Log는 현재 session의 durable 결과와 실패만 재전송합니다. Master가 재시작해 새 session이 되면 이전 session 결과는 공정 판정에 재사용하지 않습니다.
- 완성 canonical 이미지는 최근 10,000장을 유지합니다. Vision은 저장만 하고 Log만 `log.data_root` 내부의 명시적 파일 경로를 삭제합니다. 전처리 tensor/이미지는 애초에 저장하지 않습니다.
- disk 90%는 Vision warning, 95%는 신규 촬영 중지와 Master PAUSE입니다.

## 프로그램 정상 종료 산출물

`log.report_root/<session_id>/`에 다음 파일을 만듭니다.

- `product_inference_results.csv`: 제품 유입 순서, 최종 판정, A/B terminal 결과/오류, A/B model forward 시간, enqueue→결과 시간, 제품 단위 전체 시간
- `session_summary.json`: 평균 forward, 제품 시간 평균/p99.9, capture timeout 후보, GPU/VRAM 평균·peak, A/B timeout p99.9 후보

정답 label이 없으므로 confusion matrix, accuracy, precision, recall, F1은 저장하지 않습니다. 비정상 process kill은 정상 종료 보고서를 복구 생성하지 않으며 timeout 자동 조정 표본에서도 제외합니다. 원본 SQLite audit event는 남습니다.

## Timeout 자동 적용

- 기본 `log.timeout_tuning.auto_apply=false`: 후보값과 통계만 보고서에 기록합니다.
- `true`: station A/B 각각 최소 10,000개 정상 결과의 enqueue→결과 p99.9에 안전계수 1.2를 곱해 JSON을 만듭니다.
- 다음 Vision 실행도 `vision.timeout_tuning.auto_apply=true`여야 읽습니다.
- 모델 version, `.pt` SHA-256, camera/worker/model-lock을 포함한 config fingerprint가 일치하지 않으면 적용을 거부합니다.
- Linux 기본 파일은 `/var/lib/inspection/config/vision_timeout_tuning.json`입니다.

## 경로

Hardware 기본값은 DB `/var/lib/inspection/log/inspection.sqlite3`, spool `/var/lib/inspection/spool`, 이미지 `/var/lib/inspection/images`, 보고서 `/var/lib/inspection/reports`입니다. 이 폴더는 설치 단계에서 전용 Linux 사용자에게 쓰기 권한을 부여해야 합니다.

SQLite online backup, 장기 archive, retention 실행 주기 최적화는 실제 운전량과 저장장치가 확정된 뒤 남은 운영 항목입니다.

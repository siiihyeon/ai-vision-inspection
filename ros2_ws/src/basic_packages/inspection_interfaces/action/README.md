# Action 계약 메모

## CaptureProduct

성공은 frame 수신이 아니라 2448×2048 Mono8 PNG atomic 저장·digest/readability·packet_loss=0 확인과 `InferenceJob` enqueue까지 끝난 시점입니다. Queue full 동안 Action은 `ENQUEUE_BLOCKED` feedback을 반복하며 종료하지 않습니다. 취소 또는 복구 실패만 terminal failure입니다. 정상 결과는 `error_code=0`, `reason=""`입니다.

Feedback 단계는 `VALIDATING → CAMERAS_READY → TRIGGERING → WAITING_FRAMES → VALIDATING_SKEW → SAVING_FILES → ENQUEUEING_INFERENCE`이며 재촬영은 `RETRYING`, queue 포화는 `ENQUEUE_BLOCKED`로 보고합니다. `progress`는 0.0~1.0 범위입니다.

## PositionProduct

Control은 `PositionSettled` 승인 필드를 결과와 event에 동일하게 기록합니다. 현재 위치 source는 open-loop estimate입니다. 실제 위치 검증 알고리즘은 Control README의 미결정값이 필요합니다.

## ActuateProduct

Master가 잠긴 제품 결과와 Sensor3 event를 결합해 명령합니다. actuator 명령 의미, feedback 센서와 완료 시점은 미결정입니다.

## InitializeNode

같은 request ID/같은 payload는 결과 재생, 같은 ID/다른 payload는 9000 충돌입니다. 새 재시도는 새 request ID와 `retry_of_request_id`를 씁니다. 설정 digest는 SHA-256이며 노드 초기화는 직렬 실행됩니다.

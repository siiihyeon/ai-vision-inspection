# inspection_interfaces 2.1.0

노드 간 통신의 breaking baseline입니다. 숫자 enum 값과 필드명은 DB·로그·firmware adapter까지 공유되므로 단독 변경하지 않습니다.

## 계약 분류

- 공통: `CommonHeader`, `CommandHeader`, `ErrorCode`, `NodeHealth`, `SystemState`, `StationId`, `ConveyorId`, `NodeRuntimeEnvironment`
- 생존/운영: `MasterHeartbeat`, `NodeHeartbeat`, `SystemCommand`, `GetNodeStatus`, `InitializeNode`, `OperatorCommand`
- Control: `SensorEvent`, `PositionSettled`, `PositionProduct`, `ActuateProduct`
- Vision: `ImageReference`, `CaptureProduct`, `VisionQueueState`, `StationResult`, `StationInferenceFailed`, `InferenceCancellation`, `InferenceCancellationAck`
- Master: `ProductResultLocked`
- Log: `LogEvent`, `LogPersistedAck`, `ReplayStationResults`

## 불변 규칙

- `session_id`, `command_id`, process instance는 UUIDv4 문자열입니다.
- `payload_digest`는 canonical payload의 lowercase SHA-256 hex입니다.
- `ErrorCode`: 1000 capture, 2000 inference, 3000 control, 4000 actuator, 5000 log, 9000 common.
- 상태·station·conveyor·verdict는 ROS 숫자 상수와 Python `IntEnum`의 값을 일치시킵니다.
- Capture 성공 결과는 `error_code=0`, `reason=""`입니다. 실패 결과만 구체 error와 사람이 읽을 수 있는 reason을 사용합니다.
- `CaptureProduct.Result`에는 `warning_codes`가 없습니다. 경고는 `LogEvent`입니다.
- `ImageReference.file_path`는 한 PC의 공유 `data_root` 아래 절대 경로입니다. Vision은 완성 파일을 저장만 하고 Log만 승인된 보존정책으로 삭제합니다.
- `InferenceCancellation`은 A terminal NG, Sensor3 lock, session 종료를 Vision queue/worker에 전달합니다. active forward는 완료 후 결과만 폐기합니다.
- camera timestamp는 raw/domain/정규화 ns/동기화 여부까지 보존하지만, 동기화 사용 가능성이 확정되기 전에는 skew에 사용하지 않습니다. skew는 host monotonic arrival max-min입니다.

## 변경 절차

1. 송신/수신/Log 담당자가 필드와 멱등·timeout 영향을 함께 검토합니다.
2. breaking 변경이면 package major version과 `hardware.yaml` 기대 버전을 올립니다.
3. `tools/verify_skeleton.py`, Jazzy `colcon build`, endpoint 통합 test를 같은 PR에서 통과시킵니다.

## 결정 필요

Message 구조 자체는 골격에 충분합니다. Camera serial과 station 배치는 확정됐지만
Station A serial별 물리 view 역할·모델 입력 순서가 남았습니다. 그 밖에 sensor ID,
conveyor와 station mapping, 진단용 camera timestamp domain 세부 의미, model score
schema가 남았습니다. 값이 기존 필드로 표현 불가능한 경우에만 v2 계약 변경을
제안합니다. Vision 항목은 [Vision Node 완성 결정표](../../nodes/inspection_vision/README_COMPLETION_CHECKLIST.md)를
기준으로 합니다.

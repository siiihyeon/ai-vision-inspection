# inspection_master

`MasterNode`는 전체 시스템 상태, 제품 ID, 단일 활성 FIFO, 제품의 물리 위치,
Station A/B 결과 결합, Sensor3 최종 판정 잠금과 액추에이터 분류를
단독 소유합니다.

## 구현 상태

마스터 노드의 8개 블록이 연결되어 있습니다.

| 블록 | 책임 | 구현 상태 |
|---|---|---|
| 1 | 전체 시스템 FSM과 운전 명령 | 완료 |
| 2 | Control·Vision·Log 초기화와 heartbeat 감시 | 완료 |
| 3 | 단일 물리 FIFO와 Sensor1/2/3 매핑 | 완료 |
| 4 | Station A/B 위치 이동·정지 확인·촬영·재가동 | 완료 |
| 5 | 비동기 비전 결과, 재시도 결과, 늦은 결과 처리 | 완료 |
| 6 | Sensor3 판정 잠금, 액추에이터 명령, FIFO 제거 | 완료 |
| 7 | PAUSE·RESET·FAULT_STOP·LINE_CLEAR 복구 guard | 완료 |
| 8 | 판정 발행, SQLite log spool/ACK, 안전 종료 | 완료 |

`완료`는 Master 책임 범위의 코드와 ROS 연결점이 구현되었다는 뜻입니다.
Control·Vision의 실장비 adapter, Mega/TB6600 신호, MVS 카메라, 액추에이터
feedback은 각 노드 담당 구현과 통합 시험이 필요합니다.

## 핵심 운전 규칙

- Station A는 카메라 3대, Station B는 1대를 사용합니다.
- 촬영은 소프트웨어 트리거 `GIGE_ACTION_COMMAND`, LED는 외부 상시 점등입니다.
- 제품 흐름은 Master의 `ProductLedger` 하나가 소유합니다.
- Station A 촬영 후 추론을 기다리지 않고 FLIPPING으로 이동합니다.
- A/B 추론은 비동기로 진행되며 Sensor3 수락 시점에 최종 판정을 한 번만 잠금합니다.
- Sensor3에서 결과가 없거나 촬영·추론이 최종 실패한 제품은 `FORCED_NG`입니다.
- `StationResult.score`는 유한한 값이어야 합니다. `NaN`·`Inf`는 비전 결과
  계약 위반으로 기록하고 해당 station을 `FORCED_NG` 처리하며, 진단 payload에는
  `score=null`, `score_is_finite=false`를 남깁니다.
- 제품/센서/FIFO 식별 정합성을 잃으면 `FAULT_STOP + LINE_CLEAR_REQUIRED`입니다.
- 카메라·Vision 통신 단절과 같은 복구 가능 장비 오류는 `PAUSED` 후 재시도합니다.
- `RESET` 또는 worker의 `DEGRADED`·`RECOVERING` 감지 후에는 status 조회만
  반복하지 않고 해당 worker의 `InitializeNode` Action을 다시 실행합니다.
- 따라서 각 worker의 `initialize_node_resources()`는 같은 프로세스에서 여러 번
  호출되어도 안전해야 합니다. 이미 연 장치·파일·DB를 재사용하거나 새 자원으로
  교체한 뒤 이전 자원을 명시적으로 닫는 멱등 재초기화 계약을 지켜야 합니다.
- `PositionProduct` Goal 수락 전 응답 timeout은 장비가 움직이지 않은 것으로
  보고 `PAUSED`에서 복구하지만, Goal 수락 후 결과·위치를 신뢰할 수 없으면
  `FAULT_STOP + LINE_CLEAR_REQUIRED`입니다.
- 액추에이터 Goal이 수락된 뒤 완료 여부를 알 수 없으면 물리 상태가 불명하므로 `FAULT_STOP`입니다.
- 완료 제품은 활성 FIFO에서 즉시 빠지지만 late result 진단을 위해 Context를 10분 보존한 뒤 bounded tombstone으로 전환합니다.
- Master local spool 장애 시 health를 `DEGRADED`로 내리고 내구성 보장 없이
  LogNode 직접 발행을 시도합니다. spool이 없으면 초기화와 신규 START는 차단됩니다.
- Ctrl+C는 즉시 종료가 아니라 양쪽 컨베이어 정지·로그 보존 확인 후 종료합니다.
- 코드·ROS 인터페이스의 정상 판정명은 `PASS`이며, HMI에서는 같은 값을 `OK`로
  표시할 수 있습니다.
- `FAULT_STOP` 진입 뒤 늦게 도착한 Action·추론 결과와 재가동 확인은 진단
  로그만 남기고 무시하여, 고장 시점의 제품·FIFO·cycle을 복구 근거로 보존합니다.
  Vision 결과·실패 로그에는 판정, 점수, revision, 모델 버전, 실패 코드·사유와
  FrameBatch/InferenceJob 식별자를 함께 남깁니다.
- 단, `FAULT_STOP` 중 반복되는 Capture feedback과 Vision queue state는 결과가
  아닌 고빈도 telemetry이므로 상태를 바꾸지 않고 로그 없이 무시합니다.
- `LINE_CLEAR` 완료 시 이전 Vision queue pause/recovery flag도 함께 초기화하여
  다음 운전의 RESUME guard나 timeout 판단에 섞이지 않게 합니다.
- LINE_CLEAR·제자리 복구와 종료 시 Master가 보유한 Action Goal에 취소를
  요청합니다. 취소는 best-effort이며 실제 안전 정지는 RESET·PAUSE 명령과
  Control adapter의 안전 출력이 보장해야 합니다.
- 이미 회수된 station cycle의 늦은 `PositionSettled`는 경고 후 무시하며,
  살아 있는 cycle과 identity·target이 충돌할 때만 `FAULT_STOP` 처리합니다.

## 하드웨어 통합 전 남은 Control → Master 계약

`EquipmentState`는 이미 구현되어 연결되어 있습니다. Control이 상태가 바뀔
때만 발행하고, Master `_handle_equipment_state`가 안전 guard mirror 갱신과
station별 재가동 확인(`_confirm_pending_resume` → `confirm_conveyor_resumed`)에
사용합니다. 필드는 상·하층 실제 RUN/STOP, Sensor1/2/3 CLEAR, 액추에이터 안전
위치뿐이며, 작업 구역 CLEAR와 E-stop은 보고할 센서가 없어 포함하지 않습니다.

다음 계약은 아직 Control 담당자와 확정되지 않았습니다.

| 계약 | 포함해야 할 정보 | 사용 목적 |
|---|---|---|
| `EquipmentCommandResult` 성격의 완료 event | 원본 `command_id`, 명령 종류, 대상 컨베이어, 성공 여부, 실제 상태, 오류 코드·사유 | 전체 RUN/STOP/RESET 확인, 촬영 후 층별 재가동 확인 |

단순 현재 상태만 보고 층별 재가동을 확정하면 다른 명령의 결과를 잘못 연결할
수 있으므로 완료 event에는 원본 `command_id` 상관관계가 필요합니다.
Control의 장시간 Position·Actuation 실행 루프는 cancel 요청을 주기적으로
확인하고, RESET 또는 `command_epoch` 변경 시 이전 명령을 폐기해야 합니다.
Action server가 cancel 요청을 수락했다는 사실만으로 물리 정지를 확정하면 안 됩니다.

> **알려진 공백**: `EquipmentState`의 running 전이는 station별 재가동
> 확인에는 쓰이지만, 시스템 전체 START 확인(`confirm_all_conveyors_running`)에는
> 아직 연결되지 않았습니다. hardware profile은 지금도 START 시마다
> `master.action.conveyor_run_timeout_ms` 타임아웃에만 의존합니다
> (`_request_conveyor_run`). 연결 여부는 별도 결정 필요.

## 개발용 터미널 명령

HMI를 붙이기 전에는 `/inspection/master/operator_command` Service로 Master FSM을
조작합니다. `request_id: ''`를 보내면 Master가 UUID를 발급합니다.

```bash
# 1=INITIALIZE
ros2 service call /inspection/master/operator_command inspection_interfaces/srv/OperatorCommand "{request_id: '', command_type: 1, reason: 'developer initialize', operator_id: ''}"

# 2=START, 3=PAUSE, 4=RESUME, 5=RESET
ros2 service call /inspection/master/operator_command inspection_interfaces/srv/OperatorCommand "{request_id: '', command_type: 2, reason: 'developer start', operator_id: ''}"

# 6=CONFIRM_LINE_CLEAR; 신규 RUN 전/FAULT 복구 시 작업자 ID 필수
ros2 service call /inspection/master/operator_command inspection_interfaces/srv/OperatorCommand "{request_id: '', command_type: 6, reason: 'line physically cleared', operator_id: 'operator-01'}"
```

종료는 터미널에서 Ctrl+C로 요청합니다. Master가 안전 정지 확인을
마치기 전에 두 번째 Ctrl+C를 누르면 강제 종료되므로 실장비에서는
긴급한 경우가 아니면 사용하지 않습니다.

## 파일

- `inspection_master/master_node.py`: 8개 블록의 ROS 연결과 전체 조율
- `inspection_master/system_fsm.py`: 전체 시스템 상태 전이표
- `inspection_master/worker_supervision.py`: 작업 노드 초기화·heartbeat 상태
- `inspection_master/product_flow.py`: 제품 Context, 단일 FIFO, Sensor3 판정 규칙
- `inspection_master/operation_runtime.py`: 진행 중인 Action과 장비 안전 guard
- `마스터노드_읽기가이드.md`: 코드 읽기 순서와 필수/보조 함수 분류

코드를 처음 읽을 때는 `마스터노드_읽기가이드.md`부터 보시면 됩니다.

## 검증

ROS 2가 없는 환경에서 순수 도메인 계약을 검사합니다.

```bash
pyflakes ros2_ws/src ros2_ws/tools
python3 ros2_ws/tools/verify_skeleton.py
python3 ros2_ws/tools/test_domain_contracts.py
```

Ubuntu 24.04 + ROS 2 Jazzy에서는 추가로 `colcon build`과 sim/hardware 통합 시험을
수행해야 합니다.

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
Vision의 MVS Action1 adapter는 구현됐지만 Ubuntu 실장비 검증과 실제 모델
decoder가 남아 있고, Control의 Mega/TB6600·액추에이터 adapter는 placeholder입니다.
따라서 각 노드의 실장비 통합 시험이 별도로 필요합니다.

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
- Position은 Action이 아닙니다. Mega가 센서 감지 후 자율로 이동·정지하고
  `PositionSettled`만 보고하며, Master는 `cycle.deadline_ns` 안에 이 이벤트가
  안 오면 `EquipmentState`로 컨베이어가 아직 RUNNING인지 봅니다 — 여전히
  RUNNING이면 이동이 시작된 적이 없다는 뜻이라 `PAUSED`에서 복구하고,
  RUNNING을 벗어났으면 물리 위치를 신뢰할 수 없으므로 `FAULT_STOP`합니다.
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
- 이미 회수됐거나 아직 위치 대기 단계가 아닌 station cycle에 도착한 늦은
  `PositionSettled`는 경고 후 무시합니다. `conveyor_id`로만 매칭하며(한
  station엔 항상 최대 하나의 cycle만 활성이므로 모호함이 없습니다),
  Master가 명령을 보내지 않으므로 identity·target 불일치 개념이 없습니다.

## 하드웨어 통합 전 남은 Control → Master 계약

`EquipmentState`는 이미 구현되어 연결되어 있습니다. Control이 상태가 바뀔
때만 발행하고, Master `_handle_equipment_state`가 안전 guard mirror 갱신과
station별 재가동 확인(`_confirm_pending_resume` → `confirm_conveyor_resumed`)에
사용합니다. 필드는 상·하층 실제 RUN/STOP, Sensor1/2/3 CLEAR, 액추에이터 안전
위치뿐이며, 작업 구역 CLEAR와 E-stop은 보고할 센서가 없어 포함하지 않습니다.

`EquipmentState`는 depth가 낮은(`state_qos`) 상태 스냅샷이라, 같은 시각
안에 상태가 연달아 여러 번 바뀌면(예: 촬영 재가동 직후 다음 제품이 이미
센서에 대기 중이어서 RUNNING -> POSITIONING으로 곧바로 되돌아가는 경우)
중간 RUNNING 전이가 구독 콜백에 아예 전달되지 않을 수 있습니다. 이 상태로
재가동 확인이 "정지 -> 가동" rising edge에만 반응했다면, 그 전이 샘플이
유실된 순간 재가동 확인이 영영 안 와서 물리적으로는 이미 재가동된 제품이
FLIPPING으로 전이되지 않은 채 남고, 이후 Sensor2가 그 제품을 실제로
감지하면 원장에 FLIPPING 제품이 없어 `Sensor2/FIFO mismatch`로
FAULT_STOP됩니다. 대응으로 두 가지를 함께 적용했습니다:
1. `_equipment_state_subscription`의 큐 depth를 늘려(`state_qos(depth=8)`)
   빠른 연속 전이가 콜백까지 도달할 여지를 넓혔습니다.
2. `_confirm_pending_resume` 호출 조건을 rising edge에서 level 체크로
   바꿨습니다(running=True인 메시지마다 매번 확인 시도). `confirm_conveyor_resumed`가
   이미 RESUME_PENDING이 아닌 cycle은 조용히 무시하는 멱등 호출이라 안전합니다.

여기에 더해 `ConveyorResumed`(Control이 Mega의 RUN ACK을 확인하고 보내는
1회성 확정 이벤트, `conveyor_id`만 포함)를 `_handle_conveyor_resumed`가
구독해 `_confirm_pending_resume`을 똑같이 호출합니다. `EquipmentState`
전이 감지와 완전히 독립된 두 번째 경로라, 한쪽이 유실돼도 다른 쪽이
재가동을 확인할 수 있습니다.

다음 계약은 아직 Control 담당자와 확정되지 않았습니다. `ConveyorResumed`는
아래 `EquipmentCommandResult`의 축소판이 아니라, 촬영 후 층별 재가동 확인
문제만 좁게 해결하는 별도 이벤트입니다 — `command_id` 상관관계, 명령
종류, 오류 코드 등 전체 RUN/STOP/RESET 결과를 다루는 계약은 여전히
미확정입니다.

| 계약 | 포함해야 할 정보 | 사용 목적 |
|---|---|---|
| `EquipmentCommandResult` 성격의 완료 event | 원본 `command_id`, 명령 종류, 대상 컨베이어, 성공 여부, 실제 상태, 오류 코드·사유 | 전체 RUN/STOP/RESET 확인 |

단순 현재 상태만 보고 명령 결과를 확정하면 다른 명령의 결과를 잘못 연결할
수 있으므로 완료 event에는 원본 `command_id` 상관관계가 필요합니다.
Control의 장시간 Position·Actuation 실행 루프는 cancel 요청을 주기적으로
확인하고, RESET 또는 `command_epoch` 변경 시 이전 명령을 폐기해야 합니다.
Action server가 cancel 요청을 수락했다는 사실만으로 물리 정지를 확정하면 안 됩니다.
`EquipmentState`의 running 전이는 station별 재가동 확인뿐 아니라 시스템 전체
START 확인(`confirm_all_conveyors_running`)에도 쓰입니다 — 두 컨베이어가 모두
running으로 확인되면 `_handle_equipment_state`가 바로 호출하며,
`master.action.conveyor_run_timeout_ms` 타임아웃은 이 확인이 오지 않는
실제 고장 상황을 위한 안전망으로만 남습니다. 이쪽은 이미 level 체크라
`ConveyorResumed`를 적용하지 않았습니다.

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

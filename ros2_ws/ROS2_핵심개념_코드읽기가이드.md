# ROS 2 코드 개념 사전

> **v2 기준 안내:** 이 문서는 `inspection_interfaces 2.0.0`과 현재 v2 Python 골격을 기준으로 정리한 학습 자료입니다. 인터페이스 필드의 최종 정본은 `.msg`, `.srv`, `.action` 파일이며, Python 상태값의 정본은 `inspection_common/constants.py`입니다.

> AI 비전 검사 시스템의 ROS 2 코드를 읽다가 모르는 용어·함수·구조가 나왔을 때 찾아보는 사전형 문서

## 이 문서를 사용하는 방법

1. VS Code에서 `Ctrl+F`를 누른다.
2. 코드에 나온 영문 이름을 그대로 검색한다.
3. 각 항목의 `한 줄 정의 → 코드 형태 → 프로젝트 적용` 순서로 읽는다.

예를 들어 `publish_feedback`이 기억나지 않으면 `Ctrl+F`로 그대로 검색한다.

## 출처 표기

각 개념에는 다음 출처 표시를 붙인다.

| 표시 | 의미 |
|---|---|
| **ROS 2 제공** | ROS 2 또는 `rclpy`가 제공하는 개념·클래스·메서드 |
| **프로젝트 정의** | `inspection_*` 패키지에서 직접 정의한 인터페이스·클래스·상태·ID |
| **Python 제공** | Python 언어 문법 또는 표준 라이브러리 기능 |
| **일반 시스템 개념** | ROS 2 전용이 아닌 소프트웨어·하드웨어 일반 개념 |

프로젝트 정의 타입도 `.msg`, `.srv`, `.action`을 빌드하면 ROS 2가 Python 클래스를
자동 생성한다. 즉, **데이터 구조는 프로젝트가 설계하고, 실행용 클래스 생성은 ROS 2가 담당**한다.

---

# 1. 코드 키워드 빠른 색인

## 1.1 `node_base.py` 주요 키워드

| 코드 키워드 | 찾아볼 개념 | 한 줄 의미 |
|---|---|---|
| `rclpy.init()` | `rclpy` | ROS 2 Python 실행 환경 시작 |
| `Node` | `Node` | ROS Graph에 참여하는 실행 주체 |
| `super().__init__()` | `상속` | 부모 ROS Node 초기화 |
| `declare_parameter()` | `Parameter` | 노드 설정값 선언 |
| `get_parameter()` | `Parameter` | 노드 설정값 조회 |
| `create_publisher()` | `Publisher` | Topic 발행 창구 생성 |
| `publish()` | `Publisher` | Topic 메시지 실제 발행 |
| `create_subscription()` | `Subscriber` | Topic 구독과 Callback 등록 |
| `create_service()` | `Service Server` | 짧은 요청·응답 창구 생성 |
| `ActionServer()` | `Action Server` | 장시간 작업 요청 수신 창구 생성 |
| `ActionClient()` | `Action Client` | 장시간 작업 Goal 전송 창구 생성 |
| `create_timer()` | `Timer` | 주기적 Callback 등록 |
| `QoSProfile()` | `QoS` | Topic 전달·보관 정책 설정 |
| `CallbackGroup` | `Callback Group` | Callback 동시 실행 규칙 |
| `MultiThreadedExecutor` | `Executor` | 여러 Thread로 Callback 실행 |
| `spin()` | `spin` | 사건을 기다리며 Callback 처리 |
| `goal_handle` | `Goal Handle` | 실행 중인 Action Goal 관리 객체 |
| `publish_feedback()` | `Feedback` | Action 중간 진행 상황 전송 |
| `goal_handle.succeed()` | `Action 최종 상태` | Action 성공 종료 |
| `goal_handle.abort()` | `Action 최종 상태` | Action 실패 종료 |
| `goal_handle.canceled()` | `Cancel` | Action 취소 종료 |
| `async def` | `async` | 비동기 함수 선언 |
| `await` | `await` | 비동기 작업 완료까지 실행권 양보 |
| `time.monotonic_ns()` | `Monotonic Clock` | timeout 계산용 단조 증가 시계 |
| `get_clock().now()` | `ROS Clock` | 메시지 기록 시각 |
| `destroy_node()` | `Node 종료` | 노드가 소유한 ROS 자원 정리 |
| `rclpy.shutdown()` | `rclpy` | ROS 2 Python 실행 환경 종료 |

## 1.2 파일 확장자 색인

| 확장자 | 개념 | 구조 |
|---|---|---|
| `.msg` | Message | 단일 메시지 필드 정의 |
| `.srv` | Service Interface | `Request --- Response` |
| `.action` | Action Interface | `Goal --- Result --- Feedback` |
| `.launch.py` | Launch | 여러 노드와 설정을 함께 실행 |
| `.yaml` | Parameter 설정 | 노드별 설정값 주입 |
| `package.xml` | Package Manifest | 패키지 이름·버전·의존성 |
| `setup.py` | Python Package 설정 | 설치 파일과 실행 항목 정의 |

## 1.3 프로젝트 사용자 정의 타입 빠른 색인

| 사용자 정의 타입 | 종류 | 주요 구성 |
|---|---|---|
| `CommonHeader` | Message | stamp, session·message·correlation ID |
| `CommandHeader` | Message | CommonHeader, command_epoch, command_id, payload_digest |
| `MasterHeartbeat` | Message | header, command_epoch, sequence, system_state |
| `NodeHeartbeat` | Message | header, node_id, sequence, health_state, interface_version |
| `GetNodeStatus` | Service | 노드 상태 즉시 조회 |
| `InitializeNode` | Action | 노드 자원 초기화 |
| `PositionProduct` | Action | 제품 촬영 위치 이동 |
| `CaptureProduct` | Action | 동기 촬영·파일 저장·추론 Queue 등록 |
| `ActuateProduct` | Action | 정상 통과 또는 NG 분류 |
| `NodeId` | Python `StrEnum` | master, control, vision, log |
| `NodeHealthState` | Python `IntEnum` + ROS `uint8` | STARTING=0, READY=1 등 |
| `SystemState` | Python `IntEnum` + ROS `uint8` | BOOT=0, RUN_SYS=3 등 |
| `ErrorCode` | Python `IntEnum` + ROS `uint16` | 오류 종류별 고정 숫자 코드 |
| `NodeInitializationOutcome` | Python dataclass | 자원 초기화 결과 |
| `InspectionNodeBase` | Python 클래스 | 네 노드의 공통 ROS 통신·초기화 기반 |

---

# 2. 통신 방식 빠른 선택표

| 질문 | 선택 | 프로젝트 예시 |
|---|---|---|
| 계속 발생하는 상태·이벤트인가? | `Topic` | Heartbeat, 센서 이벤트, 비전 결과 |
| 짧은 질문에 즉시 응답하면 되는가? | `Service` | `GetNodeStatus` |
| 시간이 걸리고 진행 상황·취소가 필요한가? | `Action` | `InitializeNode`, 촬영, 위치 이동 |
| 노드의 실행 설정값인가? | `Parameter` | profile, heartbeat 주기, 카메라 설정 |

```text
Topic   = 방송
Service = 짧은 질문과 답변
Action  = 시간이 걸리는 작업 의뢰
```

---

# 3. ROS 2 개념 사전

## A

### Action

**출처:** ROS 2 제공

**한 줄 정의:** 시간이 걸리고 Feedback·Result·Cancel이 필요한 작업 통신 방식.

```text
Action Client → Goal → Action Server
Action Client ← Accept·Reject
Action Client ← Feedback 0회 이상
Action Client ← Result 1회
Action Client → Cancel Request 가능
```

프로젝트 예:

- `InitializeNode`
- 촬영 작업
- 촬영 위치 이동
- 액추에이터 분류 작업

공식 구조:

```text
Goal
---
Result
---
Feedback
```

같이 찾을 개념: `Action Client`, `Action Server`, `Goal`, `Goal Handle`, `Feedback`, `Result`, `Cancel`.

### Action Client

**출처:** ROS 2 제공 (`rclpy.action.ActionClient`)

**한 줄 정의:** Action Goal을 보내는 요청자.

```python
ActionClient(
    self,
    InitializeNode,
    "/inspection/vision/initialize",
)
```

현재 프로젝트에서는 Master가 주로 Action Client이다.

### Action Server

**출처:** ROS 2 제공 (`rclpy.action.ActionServer`)

**한 줄 정의:** Goal을 받고 실제 작업을 실행하는 제공자.

```python
ActionServer(
    self,
    InitializeNode,
    "vision/initialize",
    execute_callback=self._execute_initialize,
    goal_callback=self._handle_initialize_goal,
    cancel_callback=self._handle_initialize_cancel,
)
```

Control·Vision·Log는 `InitializeNode` Action Server를 제공한다. Master는 초기화 요청자이므로 해당 Server를 만들지 않는다.

### Action 최종 상태

**출처:** ROS 2 제공

**한 줄 정의:** ROS Action 자체가 성공·실패·취소 중 어떤 상태로 끝났는지 나타내는 상태.

```python
goal_handle.succeed()   # 성공
goal_handle.abort()     # 실패
goal_handle.canceled()  # 취소
```

업무 결과 필드인 `result.success`와 별개이므로 함께 설정한다.

```text
result.success=True  + goal_handle.succeed()
result.success=False + goal_handle.abort()
취소 Result           + goal_handle.canceled()
```

### async

**출처:** Python 제공

**한 줄 정의:** 기다림이 포함된 작업을 비동기 함수로 선언하는 Python 문법.

```python
async def _execute_initialize(self, goal_handle):
    outcome = await self.initialize_node_resources()
```

`async`라고 작성한다고 카메라 SDK나 AI 추론이 자동으로 병렬화되거나 빨라지는 것은 아니다.

### await

**출처:** Python 제공

**한 줄 정의:** 비동기 작업이 끝날 때까지 현재 Coroutine의 실행권을 양보하고 결과를 기다리는 문법.

```python
outcome = await self.initialize_node_resources()
```

`async/await`와 OS Thread 기반 병렬 실행은 서로 다른 개념이다.

---

## C

### CommonHeader

**출처:** 프로젝트 정의 (`inspection_interfaces/msg/CommonHeader.msg`)

**한 줄 정의:** 프로젝트 메시지에 공통으로 들어가는 추적·시각 정보.

```text
CommonHeader
├─ stamp: builtin_interfaces/Time
├─ session_id: string
├─ message_id: string
└─ correlation_id: string
```

| 멤버 | 의미 |
|---|---|
| `stamp` | 메시지 기록 시각 |
| `session_id` | 현재 운전 세션 |
| `message_id` | 개별 메시지 ID |
| `correlation_id` | 현재 메시지와 원본 요청 연결 |

### CommandHeader

**출처:** 프로젝트 정의 (`inspection_interfaces/msg/CommandHeader.msg`)

**한 줄 정의:** 장비 동작 명령의 세션·세대·멱등성·내용 무결성을 함께 검증하기 위한 v2 명령 봉투.

```text
CommandHeader
├─ header: CommonHeader
├─ command_epoch: uint64
├─ command_id: string          # UUIDv4
├─ payload_digest: string      # 명령 내용의 SHA-256
└─ issued_at: builtin_interfaces/Time
```

| 멤버 | 의미 |
|---|---|
| `command_epoch` | Reset 전후의 명령 세대를 구분 |
| `command_id` | 동일 논리 명령을 식별하고 중복 실행을 방지 |
| `payload_digest` | 같은 ID에 다른 명령 내용이 들어오는 충돌을 검출 |
| `issued_at` | 명령 발행 시각 기록 |

`PositionProduct`, `CaptureProduct`, `ActuateProduct`, `SystemCommand`가 이 구조를 사용한다.
수신 노드는 실행 전에 세션, epoch, UUID 형식, digest, 현재 시스템 상태를 검증한다.

### Callback

**출처:** 일반 프로그래밍 개념이며 ROS 2에서 폭넓게 사용

**한 줄 정의:** 특정 사건이 발생했을 때 Executor가 나중에 실행하도록 미리 등록한 함수.

```python
self.create_timer(
    0.5,
    self._publish_heartbeat,
)
```

괄호 차이:

```python
self._publish_heartbeat()  # 지금 즉시 실행
self._publish_heartbeat    # 함수 자체를 Callback으로 전달
```

Callback 종류:

| 사건 | Callback 예시 |
|---|---|
| Timer 만료 | `_publish_heartbeat()` |
| Topic 메시지 도착 | `_handle_master_heartbeat(message)` |
| Service 요청 도착 | `_handle_get_status(request, response)` |
| Action Goal 도착 | `_handle_initialize_goal(goal_request)` |
| Action 실행 | `_execute_initialize(goal_handle)` |
| Action 취소 요청 | `_handle_initialize_cancel(goal_handle)` |

### Callback Group

**출처:** ROS 2 제공

**한 줄 정의:** Callback들의 동시 실행 허용 범위를 정하는 그룹.

#### `MutuallyExclusiveCallbackGroup`

같은 그룹의 Callback은 동시에 실행하지 않는다.

```text
Callback A 실행 중 → 같은 그룹 Callback B 대기
```

#### `ReentrantCallbackGroup`

같은 그룹의 Callback도 동시에 실행될 수 있다. 공유 딕셔너리, 카메라 SDK,
시리얼 포트 등을 동시에 건드리면 Lock이나 직렬화 정책이 필요하다.

### Cancel

**출처:** ROS 2 Action 제공

**한 줄 정의:** 실행 중인 Action 작업을 중단하도록 요청하는 기능.

```python
return CancelResponse.ACCEPT
```

취소 요청 수락은 실제 장비 중단 완료를 의미하지 않는다. 노드가 안전하게 카메라,
Mega, 모터 작업을 중단하고 다음 상태로 이동해야 한다.

### Client

**출처:** ROS 2 제공 개념

**한 줄 정의:** Service Request 또는 Action Goal을 보내는 요청자.

```text
Service Client → Service Server
Action Client  → Action Server
```

---

## D

### Digest

**출처:** 프로젝트 정의 (`inspection_common.digest`), 해시 알고리즘은 Python 표준 라이브러리 제공

**한 줄 정의:** 명령이나 설정 내용을 일정한 JSON 문자열로 만든 뒤 계산한 SHA-256 지문.

```python
digest = payload_digest({"command_type": command_type, "epoch": command_epoch})
```

딕셔너리 키 순서나 JSON 공백이 달라도 같은 논리 내용이면 같은 digest가 나오도록
`canonical_json()`이 정렬·압축한 뒤 해시한다. 보안 비밀번호 저장 목적이 아니라
**같은 ID의 명령 내용이 실제로 같은지** 확인하는 데 사용한다.

### IdempotencyStore

**출처:** 프로젝트 정의 (`inspection_common.idempotency.IdempotencyStore`)

**한 줄 정의:** 같은 명령이 재전송되어도 장비 동작은 한 번만 수행하도록 기존 결과를 기억하는 제한 용량 저장소.

```text
inspect(command_id, digest)
├─ NEW      → 처음 본 명령이므로 실행 가능
├─ REPLAY   → 같은 ID와 같은 내용이므로 기존 결과 재사용
└─ CONFLICT → 같은 ID인데 내용이 다르므로 실행 거부
```

`remember()`로 완료 결과를 저장하며, 용량을 넘으면 가장 오래 사용하지 않은 항목부터 제거한다.
내부 `RLock`으로 여러 Callback이 접근할 때 저장소 자체의 자료구조를 보호한다.

---

## E

### ErrorCode

**출처:** 프로젝트 정의 (`inspection_common.constants.ErrorCode`)

**한 줄 정의:** 공통 Node 골격에서 사용하는 구조화된 오류 코드 열거형.

```text
ErrorCode
├─ NONE = 0
├─ CAPTURE_FAILED = 1000
├─ INFERENCE_FAILED = 2000
├─ CONTROL_FAILED = 3000
├─ ACTUATOR_FAILED = 4000
├─ LOG_COMMIT_FAILED = 5000
├─ COMMAND_CONFLICT = 9000
├─ INTERFACE_VERSION_MISMATCH = 9001
└─ NODE_INIT_FAILED = 9002
```

Python에서는 `IntEnum`으로 이름을 사용하고, ROS 메시지에서는 같은 값을 `uint16`으로 전송한다.
코드에서 `9000` 같은 숫자를 직접 쓰지 말고 `ErrorCode.COMMAND_CONFLICT`처럼 이름으로 사용한다.

### Endpoint

**출처:** ROS 2 제공 개념

**한 줄 정의:** Topic·Service·Action이 사용하는 이름 있는 통신 창구.

예:

```text
/inspection/master/heartbeat
/inspection/vision/get_status
/inspection/vision/initialize
```

### Executor

**출처:** ROS 2 제공

**한 줄 정의:** ROS 사건을 기다렸다가 연결된 Callback을 실제로 실행하는 관리자.

```python
executor = MultiThreadedExecutor(num_threads=4)
executor.add_node(node)
executor.spin()
```

Executor가 처리하는 사건:

- Topic 메시지 도착
- Service 요청 도착
- Action Goal·Cancel 도착
- Timer 만료

`MultiThreadedExecutor`를 사용해도 Callback Group이 병행을 금지하면 해당 Callback은 동시에 실행되지 않는다.

---

## F

### Feedback

**출처:** ROS 2 Action 제공. `stage`, `attempt` 필드는 프로젝트 정의

**한 줄 정의:** Action 작업 중 Client에게 보내는 중간 진행 상황.

`InitializeNode`의 Feedback 필드:

```text
InitializeNode.Feedback
├─ stage: string
└─ attempt: uint32
```

Python 사용:

```python
feedback = InitializeNode.Feedback()
feedback.stage = "VALIDATING"
feedback.attempt = 1
goal_handle.publish_feedback(feedback)
```

속성·함수 구분:

```text
feedback.stage             → Feedback 객체의 속성
feedback.attempt           → Feedback 객체의 속성
goal_handle.publish_feedback() → Goal Handle의 메서드
```

Feedback은 여러 번 보낼 수 있고 Result는 작업 종료 시 한 번 반환한다.

---

## G

### GetNodeStatus

**출처:** 프로젝트 정의 (`inspection_interfaces/srv/GetNodeStatus.srv`)

**한 줄 정의:** 특정 노드가 현재 세션에서 준비됐는지 짧게 조회하는 사용자 정의 Service.

```text
GetNodeStatus
├─ Request
│  ├─ request_id
│  └─ session_id
│
└─ Response
   ├─ ready
   ├─ node_id
   ├─ node_instance_id
   ├─ health_state: uint8
   ├─ interface_version
   ├─ software_version
   ├─ active_session_id
   ├─ command_epoch
   ├─ heartbeat_sequence
   ├─ master_heartbeat_alive
   ├─ uptime_ms
   └─ status_json
```

`ready`는 단순히 프로세스가 실행 중이라는 뜻이 아니다. v2 공통 구현에서는 노드가 `READY`이고,
요청 세션이 현재 세션과 일치하며, Master heartbeat도 살아 있을 때만 `True`가 된다.

Python에서 다음 타입으로 접근한다.

```python
GetNodeStatus.Request
GetNodeStatus.Response
```

### Goal

**출처:** ROS 2 Action 제공. Goal 내부 필드는 각 프로젝트가 정의

**한 줄 정의:** Action Client가 Server에 보내는 작업 요청 데이터.

```python
goal = InitializeNode.Goal()
goal.request_id = "init-vision-001"
goal.session_id = "session-001"
```

Goal이 도착하면 Action Server는 먼저 `ACCEPT` 또는 `REJECT`를 결정한다.

```python
return GoalResponse.ACCEPT
return GoalResponse.REJECT
```

`ACCEPT`는 작업 성공이 아니라 요청을 처리하겠다는 뜻이다.

### Goal Handle

**출처:** ROS 2 제공 (`rclpy` Action Server 측 객체)

**한 줄 정의:** 현재 실행 중인 Action Goal 하나의 상태·요청·Feedback·취소를 관리하는 객체.

주요 멤버 구조:

```text
goal_handle
├─ request                 Goal 데이터
├─ is_cancel_requested     취소 요청 여부
├─ publish_feedback()      중간 Feedback 발행
├─ succeed()               성공 종료
├─ abort()                 실패 종료
└─ canceled()              취소 종료
```

```python
goal_handle.request
goal_handle.publish_feedback(feedback)
goal_handle.is_cancel_requested
goal_handle.succeed()
goal_handle.abort()
goal_handle.canceled()
```

`goal_handle.request`는 변수명이 Request이지만 실제로는 Action Goal의 필드 데이터이다.

---

## H

### Heartbeat

**출처:** 프로젝트 정의 애플리케이션 기능

**한 줄 정의:** 노드가 살아 있음을 주기적으로 알리는 소프트웨어 생존신호.

```text
MasterHeartbeat → Master가 Control·Vision·Log에 발행
NodeHeartbeat   → Control·Vision·Log가 Master에 발행
```

Heartbeat를 받았다는 사실만으로 계속 정상이라고 판단할 수는 없다. 마지막 수신
monotonic 시각과 현재 시각의 차이를 timeout과 비교해야 한다.

```python
self.last_master_heartbeat_monotonic_ns = time.monotonic_ns()
```

Heartbeat는 프로젝트 애플리케이션 기능이며 QoS의 liveliness 기능과 별개로 구현한다.

### heartbeat_qos()

**출처:** 프로젝트 정의 함수. 내부 QoS 타입과 정책 값은 ROS 2 제공

**한 줄 정의:** 모든 Heartbeat Publisher·Subscriber가 같은 QoS를 사용하도록 만든 공통 보조 함수.

```text
heartbeat_qos()
└─ QoSProfile
   ├─ history = KEEP_LAST
   ├─ depth = 1
   ├─ reliability = BEST_EFFORT
   └─ durability = VOLATILE
```

### reliable_event_qos()

**출처:** 프로젝트 정의 함수. 내부 QoS 타입과 정책 값은 ROS 2 제공

**한 줄 정의:** 센서·비전 결과·로그·명령처럼 유실시키면 안 되는 업무 이벤트용 QoS.

```text
RELIABLE + VOLATILE + KEEP_LAST(depth, 기본 100)
```

### state_qos()

**출처:** 프로젝트 정의 함수. 내부 QoS 타입과 정책 값은 ROS 2 제공

**한 줄 정의:** 늦게 구독한 노드도 최신 상태 한 건을 즉시 받을 수 있게 하는 상태 Topic용 QoS.

```text
RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(1)
```

---

## I

### IntEnum

**출처:** Python 표준 라이브러리 제공 (`enum.IntEnum`). 구체적인 상태 이름과 숫자는 프로젝트 정의

**한 줄 정의:** 코드에서는 의미 있는 이름으로 쓰면서 ROS 메시지에는 정수로 전송할 수 있는 숫자형 열거형.

```python
class NodeHealthState(IntEnum):
    STARTING = 0
    READY = 1
    DEGRADED = 2
```

```python
self.health_state = NodeHealthState.READY  # 코드에서는 이름으로 읽음
message.health_state = int(self.health_state)  # ROS uint8 필드에는 1 전송
received = NodeHealthState(message.health_state)  # 받은 1을 READY로 복원
```

문자열보다 오타 가능성이 작고 Python·C++·ROS IDL이 같은 고정 번호를 공유할 수 있다.
단, 코드에서 `1`을 직접 비교하지 말고 반드시 `NodeHealthState.READY`처럼 이름을 사용한다.

### InspectionNodeBase

**출처:** 프로젝트 정의 (`inspection_common.node_base.InspectionNodeBase`)

**한 줄 정의:** 네 실행 노드에 공통 Heartbeat·상태조회·초기화 Action·실행 방식을 제공하는 기반 클래스.

```text
InspectionNodeBase
├─ 주요 상태 속성
│  ├─ node_id
│  ├─ node_instance_id
│  ├─ interface_version
│  ├─ software_version
│  ├─ profile
│  ├─ health_state
│  ├─ session_id
│  ├─ config_version
│  ├─ config_digest
│  ├─ command_epoch
│  ├─ system_state
│  ├─ _heartbeat_sequence
│  └─ last_master_heartbeat_monotonic_ns
│
├─ ROS 통신 객체
│  ├─ _heartbeat_publisher
│  ├─ _status_service
│  ├─ _heartbeat_timer
│  ├─ _master_watchdog_timer
│  ├─ _master_heartbeat_subscription
│  ├─ _system_command_subscription
│  └─ _initialize_action_server (Control·Vision·Log만 생성)
│
└─ 주요 메서드
   ├─ set_health_state()
   ├─ validate_command_header()
   ├─ required_hardware_parameters()
   ├─ validate_hardware_profile()
   ├─ initialize_node_resources()
   ├─ _publish_heartbeat()
   ├─ _handle_master_heartbeat()
   ├─ _handle_system_command()
   ├─ _check_master_heartbeat()
   ├─ _handle_get_status()
   ├─ _handle_initialize_goal()
   ├─ _handle_initialize_cancel()
   ├─ _status_snapshot()
   ├─ _fill_initialize_result()
   └─ _execute_initialize()
```

ROS 2의 `rclpy.node.Node`를 상속한 프로젝트 클래스이며, 각 담당 노드는 다시
`InspectionNodeBase`를 상속한다.

### InitializeNode

**출처:** 프로젝트 정의 (`inspection_interfaces/action/InitializeNode.action`)

**한 줄 정의:** Master가 Control·Vision·Log에 실제 장비·모델·저장소 준비를 요청하는 프로젝트 사용자 정의 Action.

ROS 2 기본 Action이 아니라 `inspection_interfaces/action/InitializeNode.action`에서 직접 정의한 인터페이스이다.

#### 전체 멤버 구조

```text
InitializeNode
├─ Goal
│  ├─ request_id
│  ├─ retry_of_request_id
│  ├─ session_id
│  ├─ config_version
│  ├─ config_digest
│  └─ expected_interface_version
│
├─ Result
│  ├─ success
│  ├─ node_id
│  ├─ error_code
│  ├─ reason
│  ├─ interface_version
│  ├─ software_version
│  ├─ request_id
│  ├─ active_session_id
│  ├─ runtime_environment
│  ├─ status_snapshot
│  └─ retryable
│
└─ Feedback
   ├─ stage
   └─ attempt
```

Python에서 ROS 2 빌드 후 다음 세 클래스로 접근한다.

```python
InitializeNode.Goal
InitializeNode.Result
InitializeNode.Feedback
```

#### Python 생성자와 구분

```text
__init__()      → ROS 객체·통신 창구·내부 변수 생성
InitializeNode  → 실제 장비·모델·DB 준비
INITIALIZING    → Master 전체 시스템 FSM 상태
```

#### 노드별 실제 초기화

| 노드 | 초기화 내용 |
|---|---|
| Control | Mega 연결, 통신 확인, 센서·모터·액추에이터 안전 상태 |
| Vision | 카메라 연결, 모델 로드, GPU·추론 Queue 준비 |
| Log | SQLite, 스키마, 이미지·spool 경로, 저장 용량 |

#### Goal 필드

| 필드 | 의미 |
|---|---|
| `request_id` | 초기화 명령 식별과 멱등 처리 |
| `retry_of_request_id` | 새 초기화 요청이 이전 어느 요청의 재시도인지 연결 |
| `session_id` | 현재 운전 세션 구분 |
| `config_version` | 적용 설정 버전 |
| `config_digest` | 설정 내용 해시 |
| `expected_interface_version` | Master가 기대하는 인터페이스 버전 |

#### Result 필드

| 필드 | 의미 |
|---|---|
| `success` | 업무 초기화 성공 여부 |
| `node_id` | 응답 노드 ID |
| `error_code` | 구조화된 오류 코드 |
| `reason` | 사람이 읽는 실패·성공 설명 |
| `interface_version` | 실제 인터페이스 버전 |
| `software_version` | 실제 노드 패키지 버전 |
| `request_id` | 어떤 초기화 요청에 대한 결과인지 재확인 |
| `active_session_id` | 초기화 이후 노드가 실제로 적용한 세션 |
| `runtime_environment` | OS·Python·ROS 배포판·SDK·펌웨어 버전 |
| `status_snapshot` | JSON 상태 상세 |
| `retryable` | 자동 재시도 가능 여부 |

#### Feedback 필드

| 필드 | 의미 |
|---|---|
| `stage` | 현재 초기화 단계 |
| `attempt` | 현재 시도 횟수 |

공통 검증은 `InspectionNodeBase._execute_initialize()`가 담당하고, 실제 노드별 자원 준비는 `initialize_node_resources()`가 담당한다.

### PositionProduct

**출처:** 프로젝트 정의 (`inspection_interfaces/action/PositionProduct.action`)

**한 줄 정의:** Master가 Control에 특정 제품을 지정 스테이션 촬영 위치로 이동시키도록 요청하는 Action.

```text
PositionProduct
├─ Goal: command, product_id, fifo_sequence, station_id, conveyor_id, target_step
├─ Result: success, product_id, station_id, position_command_id,
│          estimated_step, position_error_steps, position_source, error_code, reason
└─ Feedback: stage, estimated_step
```

### CaptureProduct

**출처:** 프로젝트 정의 (`inspection_interfaces/action/CaptureProduct.action`)

**한 줄 정의:** Master가 Vision에 제품 촬영부터 파일 검증·저장·추론 Queue 등록까지 요청하는 Action.

```text
CaptureProduct
├─ Goal
│  ├─ command
│  ├─ product_id, fifo_sequence, station_id, capture_id
│  ├─ required_camera_ids[]
│  └─ requested_at
├─ Result
│  ├─ success, product_id, station_id, capture_id, frame_batch_id
│  ├─ attempt_count, images[], frame_arrival_skew_us, inference_job_id
│  └─ error_code, reason
└─ Feedback
   ├─ stage, attempt, frame_batch_id
   ├─ completed_camera_ids[], pending_camera_ids[]
   └─ progress, reason
```

이 Action의 성공은 **추론 완료가 아니라** 필요한 이미지가 저장·검증되고 추론 Queue에 등록됐다는 뜻이다.
Station 결과는 나중에 `StationResult` 또는 `StationInferenceFailed` Topic으로 비동기 도착한다.

### ActuateProduct

**출처:** 프로젝트 정의 (`inspection_interfaces/action/ActuateProduct.action`)

**한 줄 정의:** Sensor3에 매핑된 제품을 정상 통과 또는 NG 배출하도록 Control에 요청하는 Action.

```text
ActuateProduct
├─ Goal: command, product_id, fifo_sequence, sensor3_event_id, actuator_command
├─ Result: success, product_id, actuation_id, error_code, reason
└─ Feedback: stage
```

`actuator_command`는 `DIVERT_NG=1`, `PASS_THROUGH=2` 중 하나이다.

### Interface

**출처:** ROS 2 제공 방식. 실제 필드 구성은 프로젝트 정의

**한 줄 정의:** 노드 사이에서 교환할 데이터 구조를 정의한 통신 계약.

| 파일 | 구조 | Python 생성 타입 |
|---|---|---|
| `.msg` | 단일 메시지 | `NodeHeartbeat` 등 |
| `.srv` | Request + Response | `GetNodeStatus.Request/Response` |
| `.action` | Goal + Result + Feedback | `InitializeNode.Goal/Result/Feedback` |

인터페이스 파일은 빌드 시 Python·C++ 타입으로 자동 생성된다.

```bash
ros2 interface show inspection_interfaces/action/InitializeNode
```

---

## L

### DurableLogSpool

**출처:** 프로젝트 정의 (`inspection_common.log_spool.DurableLogSpool`)

**한 줄 정의:** LogNode의 SQLite commit ACK가 오기 전까지 송신 로그를 로컬 SQLite에 보존하는 내구성 Queue.

```text
enqueue()     → log_id + revision + payload를 보존
pending()     → 아직 ACK되지 않은 로그 조회
acknowledge() → LogPersistedAck를 받은 항목만 삭제
close()       → SQLite 연결 종료
```

Topic을 `RELIABLE`로 설정해도 상대 프로세스 장애나 재시작까지 영구 보존되는 것은 아니므로,
중요 로그는 통신 QoS와 별도로 이 spool에 저장한다.

### Launch

**출처:** ROS 2 제공

**한 줄 정의:** 여러 노드와 Parameter·Namespace를 한 번에 실행하는 ROS 2 실행 구성.

개별 실행:

```bash
ros2 run inspection_master master_node
```

전체 실행:

```bash
ros2 launch inspection_bringup inspection_system.launch.py profile:=sim
```

현재 `inspection_bringup` 패키지가 네 노드와 공통 YAML 설정을 함께 실행한다.

---

## M

### MasterNode

**출처:** 프로젝트 정의 (`inspection_master`)

**한 줄 정의:** 제품 FIFO와 전체 시스템 상태를 소유하고 다른 노드를 조율하는 프로젝트 노드.

ROS 2 자체의 중앙 `ROS Master`가 아니다. ROS 2는 중앙 Master 없이 노드들이 분산 검색으로 서로를 발견한다.

현재 v2 골격의 주요 멤버 구조:

```text
MasterNode
├─ 부모 기능: InspectionNodeBase
├─ session_id: Master 시작 시 생성한 UUID
├─ ledger: ProductLedger
│  └─ product_id + fifo_sequence 기준 A/B 결과와 판정 잠금 관리
├─ result_reorder: ProductResultReorderBuffer
│  └─ 비동기 결과를 fifo_sequence 순서로 재정렬
├─ worker_heartbeats
│  └─ NodeId별 최신 NodeHeartbeat
├─ worker_received_ns
│  └─ worker별 마지막 heartbeat 수신 monotonic 시각
├─ _worker_heartbeat_subscriptions
│  ├─ Control Heartbeat Subscription
│  ├─ Vision Heartbeat Subscription
│  └─ Log Heartbeat Subscription
├─ initialize_clients
│  ├─ Control InitializeNode ActionClient
│  ├─ Vision InitializeNode ActionClient
│  └─ Log InitializeNode ActionClient
├─ position_client: Control PositionProduct ActionClient
├─ capture_client: Vision CaptureProduct ActionClient
├─ actuate_client: Control ActuateProduct ActionClient
├─ 업무 Subscription
│  ├─ StationResult / StationInferenceFailed
│  ├─ SensorEvent / PositionSettled
│  ├─ VisionQueueState
│  └─ LogPersistedAck
├─ 업무 Publisher
│  ├─ ProductResultLocked
│  ├─ SystemCommand
│  └─ LogEvent
└─ 구현 블록
   ├─ BLOCK 1: 전체 시스템 FSM·운전 명령
   ├─ BLOCK 2: 작업 노드 초기화·생존 감시
   ├─ BLOCK 3: 단일 물리 FIFO·Sensor1/2/3 매핑
   ├─ BLOCK 4: Station A/B 위치 이동·촬영
   ├─ BLOCK 5: 비동기 비전 결과·최종 판정
   ├─ BLOCK 6: Sensor3 액추에이터·FIFO 제거
   ├─ BLOCK 7: PAUSE·RESET·FAULT_STOP 복구
   └─ BLOCK 8: 판정 발행·로그 spool/ACK·종료
```

현재 각 블록의 함수명과 호출 경계가 만들어졌고, BLOCK 1의 순수 상태 전이 규칙은
`inspection_master/system_fsm.py`에 구현돼 있다. START·RESUME·RESET의 장비 guard는 뒤 블록이
완성될 때까지 fail-closed로 막혀 있다. 블록별 구현 상태는 `inspection_master/README.md`를 기준으로 확인한다.

### MasterHeartbeat

**출처:** 프로젝트 정의 (`inspection_interfaces/msg/MasterHeartbeat.msg`)

**한 줄 정의:** Master의 생존, 명령 세대, 전체 시스템 상태를 작업 노드에 알리는 Message.

```text
MasterHeartbeat
├─ header: CommonHeader
│  ├─ stamp
│  ├─ session_id
│  ├─ message_id
│  └─ correlation_id
├─ master_instance_id: string
├─ interface_version: string
├─ command_epoch: uint64
├─ sequence: uint64
└─ system_state: uint8
```

`system_state`는 문자열이 아니라 `SystemState`와 숫자를 공유한다. 예를 들어
`SystemState.RUN_SYS`는 Python에서 이름으로 사용하고 메시지에는 `3`이 들어간다.

### Message

**출처:** ROS 2 제공 방식. 구체적인 Message 필드는 프로젝트 정의 가능

**한 줄 정의:** Topic 등을 통해 전송하는 정해진 필드 구조의 데이터 객체.

```python
message = NodeHeartbeat()
message.node_id = "vision"
message.health_state = int(NodeHealthState.READY)
```

`.msg` 파일을 빌드하면 Python Message 클래스가 자동 생성된다.

### Monotonic Clock

**출처:** Python 표준 라이브러리 제공 (`time.monotonic_ns`)

**한 줄 정의:** 시스템 시각 변경과 무관하게 앞으로만 증가하는 경과 시간 측정용 시계.

```python
time.monotonic_ns()
```

Heartbeat timeout, 명령 timeout, 작업 경과 시간 계산에 사용한다. 사람이 읽는 실제 날짜·시간이 아니다.

### MultiThreadedExecutor

**출처:** ROS 2 제공 (`rclpy.executors`)

**한 줄 정의:** 여러 Thread를 사용해 허용된 Callback들을 병행 처리하는 Executor.

```python
MultiThreadedExecutor(num_threads=4)
```

동시에 실행 가능한지는 Callback Group의 설정도 함께 결정한다.

---

## N

### Namespace

**출처:** ROS 2 제공

**한 줄 정의:** 노드·Topic·Service·Action 이름을 공통 경로 아래 묶는 이름 공간.

```python
namespace="inspection"
```

상대 이름:

```text
master/heartbeat
```

Namespace 적용 후 전체 이름:

```text
/inspection/master/heartbeat
```

### Node

**출처:** ROS 2 제공 (`rclpy.node.Node`)

**한 줄 정의:** ROS Graph에 참여해 Publisher·Subscriber·Service·Action·Parameter를 소유하는 논리적 실행 주체.

```python
class InspectionNodeBase(Node):
    ...
```

노드는 같은 프로세스, 다른 프로세스 또는 다른 컴퓨터에서 통신할 수 있다.

### NodeHeartbeat

**출처:** 프로젝트 정의 (`inspection_interfaces/msg/NodeHeartbeat.msg`)

**한 줄 정의:** Control·Vision·Log의 생존과 Health·인터페이스 버전을 Master에 알리는 Message.

```text
NodeHeartbeat
├─ header: CommonHeader
│  ├─ stamp
│  ├─ session_id
│  ├─ message_id
│  └─ correlation_id
├─ node_id: string
├─ node_instance_id: string
├─ sequence: uint64
├─ health_state: uint8
└─ interface_version: string
```

`node_instance_id`는 같은 노드 프로세스가 재시작됐는지를 구분하는 UUID이다.
`health_state`는 `NodeHealthState`의 고정 숫자값이며 로그에서는 Enum 이름으로 변환해 읽는다.

### NodeId

**출처:** 프로젝트 정의 (`inspection_common.constants.NodeId`)

**한 줄 정의:** 네 실행 노드를 고정된 문자열로 구분하는 `StrEnum`.

```text
NodeId
├─ MASTER  = "master"
├─ CONTROL = "control"
├─ VISION  = "vision"
└─ LOG     = "log"
```

### NodeInitializationOutcome

**출처:** 프로젝트 정의 (`inspection_common.node_base.NodeInitializationOutcome`)

**한 줄 정의:** 자식 노드의 실제 자원 초기화 결과를 공통 Action 처리에 전달하는 dataclass.

```text
NodeInitializationOutcome
├─ success: bool
├─ reason: str
├─ retryable: bool = False
├─ error_code: int = int(ErrorCode.NONE)
└─ status_details: dict[str, object] = field(default_factory=dict)
```

Control·Vision·Log의 `initialize_node_resources()`가 이 객체를 반환한다.

### Node Health State

**출처:** 프로젝트 정의 (`inspection_common.constants.NodeHealthState`)

**한 줄 정의:** 개별 노드 프로그램의 준비·복구·오류 상태.

```text
NodeHealthState
├─ STARTING = 0
├─ READY = 1
├─ DEGRADED = 2
├─ RECOVERING = 3
├─ INIT_BLOCKED = 4
└─ FAULT = 5
```

Python에서는 `IntEnum`, ROS 메시지에서는 `uint8`로 표현한다. 전체 시스템 상태인
`RUN_SYS`, `PAUSED`, `FAULT_STOP`과는 역할이 다르다.

### 프로젝트 숫자 Enum 요약

**출처:** 프로젝트 정의 (`inspection_common.constants`)

| Enum | 답하는 질문 | 주요 값 |
|---|---|---|
| `SystemState` | 전체 시스템이 어느 운전 단계인가? | `BOOT`, `READY`, `RUN_SYS`, `PAUSED`, `FAULT_STOP` |
| `PauseReason` | 왜 시스템을 멈추거나 복구하는가? | `OPERATOR`, `DEVICE_RECOVERY_AUTO`, `STORAGE_RECOVERY` |
| `ProductPhysicalState` | 개별 제품이 물리 흐름의 어디에 있는가? | `STATION_A_WAIT`, `FLIPPING`, `SENSOR3_WAIT`, `DONE` |
| `PhysicalZone` | 제품이 어느 큰 물리 구간에 있는가? | `UPPER_INSPECTION`, `FLIP_TRANSFER`, `LOWER_INSPECTION` |
| `StationId` | 어느 검사 스테이션인가? | `A=1`, `B=2` |
| `ConveyorId` | 어느 컨베이어인가? | `UPPER=1`, `LOWER=2` |
| `Verdict` | 제품 판정은 무엇인가? | `PASS=1`, `NG=2`, `FORCED_NG=3` |

모두 코드에서는 이름으로 사용하고 ROS의 `uint8` 필드에 넣을 때 `int(enum_value)`로 변환한다.

### Node 종료

**출처:** ROS 2 제공 메서드와 프로젝트 종료 순서

**한 줄 정의:** 노드와 Executor가 소유한 ROS 자원을 안전하게 정리하는 과정.

```python
executor.shutdown()
node.destroy_node()
rclpy.shutdown()
```

---

## P

### ProductLedger

**출처:** 프로젝트 정의 (`inspection_master.product_flow.ProductLedger`)

**한 줄 정의:** Master가 제품 ID·FIFO 순번·A/B 검사 결과·최종 판정 잠금을 관리하는 메모리 원장.

```text
ProductLedger
├─ product_id → ProductContext
└─ fifo_sequence → product_id

ProductContext
├─ product_id
├─ fifo_sequence
├─ stations: StationId → StationDecision
└─ locked: LockedProduct | None
```

`register()`는 같은 `product_id`가 다른 순번에 붙거나 같은 순번이 다른 제품에 붙는 것을 거부한다.
`get(product_id, fifo_sequence)`는 두 식별자가 모두 일치할 때만 제품을 반환한다.

### ProductResultReorderBuffer

**출처:** 프로젝트 정의 (`inspection_master.product_flow.ProductResultReorderBuffer`)

**한 줄 정의:** A/B 추론 결과가 순서 없이 도착해도 최종 결과는 `fifo_sequence` 순서로만 내보내는 버퍼.

```text
3번 결과 먼저 도착 → 보관
1번 결과 도착      → 1번 출력
2번 결과 도착      → 2번과 대기 중이던 3번을 연속 출력
```

이 버퍼는 물리 제품 FIFO 자체가 아니라 **확정 결과의 출력 순서를 맞추는 보조 버퍼**이다.

### Product 판정 잠금

**출처:** 프로젝트 정의 (`inspection_master.product_flow.ProductContext`)

**한 줄 정의:** 한 번 확정된 제품 판정을 늦게 도착한 비전 결과가 바꾸지 못하게 만드는 규칙.

```text
A PASS + B PASS       → PASS
A/B 중 하나라도 NG   → NG
명시적 추론 실패      → 즉시 FORCED_NG
Sensor3 도착 시 미완료 → FORCED_NG
잠금 이후 늦은 결과   → 무시
```

판정이 먼저 잠겼더라도 실제 액추에이터 명령은 해당 제품의 Sensor3 이벤트 매핑이 확인된 후 내려야 한다.

### Package

**출처:** ROS 2 제공 구조

**한 줄 정의:** ROS 2 코드·인터페이스·설정의 빌드·설치·배포 단위.

현재 패키지:

```text
inspection_interfaces
inspection_common
inspection_bringup
inspection_master
inspection_control
inspection_vision
inspection_log
```

### Parameter

**출처:** ROS 2 제공

**한 줄 정의:** 노드가 소유하는 실행 설정값.

```python
self.declare_parameter("profile", "sim")
self.profile = self.get_parameter("profile").value
```

예:

```text
profile = sim 또는 hardware
comm.node_heartbeat_period_ms = 500
camera.exposure_us = 5000
mega.serial_port = /dev/ttyACM0
```

YAML에서 시작 값을 주입할 수 있다. Parameter는 시스템 상태가 아니라 설정이다.
값을 한 번 읽어 내부 변수에 저장하면 Parameter 변경이 자동 반영되지 않으므로,
동적 변경에는 별도 Parameter Callback이 필요하다.

### Publisher

**출처:** ROS 2 제공

**한 줄 정의:** 특정 Topic으로 Message를 발행하는 송신 객체.

Publisher 생성:

```python
publisher = self.create_publisher(
    NodeHeartbeat,
    "vision/heartbeat",
    heartbeat_qos(),
)
```

실제 발행:

```python
publisher.publish(message)
```

`create_publisher()`는 창구 생성이고 `publish()`가 실제 전송이다.

---

## Q

### QoS

**출처:** ROS 2 제공

**한 줄 정의:** Topic Message의 전달·보관 방식을 정하는 Quality of Service 정책.

Publisher와 Subscriber의 QoS가 호환되지 않으면 Topic 이름과 타입이 같아도 통신되지 않을 수 있다.

현재 코드에서 주로 사용하는 멤버 구조:

```text
QoSProfile
├─ history
│  ├─ KEEP_LAST
│  └─ KEEP_ALL
├─ depth
├─ reliability
│  ├─ RELIABLE
│  └─ BEST_EFFORT
└─ durability
   ├─ VOLATILE
   └─ TRANSIENT_LOCAL
```

#### History

```text
KEEP_LAST → 최근 메시지를 depth만큼 보관
KEEP_ALL  → 가능한 모든 메시지 보관 시도
```

#### Depth

```text
depth=1  → 최근 메시지 1개
depth=10 → 최근 메시지 10개
```

#### Reliability

```text
RELIABLE    → 전달 신뢰성을 높이고 재전송 시도
BEST_EFFORT → 전달을 시도하지만 완전한 재전송을 보장하지 않음
```

#### Durability

```text
VOLATILE        → 연결 전 과거 메시지는 전달하지 않음
TRANSIENT_LOCAL → 늦게 연결한 Subscriber에 보관 메시지 전달 가능
```

#### 현재 Heartbeat QoS

```python
QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
```

최신 생존신호 하나만 중요하고 한 번 유실돼도 다음 신호가 곧 오기 때문에 사용한다.

---

## R

### rclpy

**출처:** ROS 2 제공 Python Client Library

**한 줄 정의:** Python에서 ROS 2 Node와 통신 기능을 사용할 수 있게 하는 Client Library.

```python
import rclpy
rclpy.init()
...
rclpy.shutdown()
```

### Request

**출처:** ROS 2 Service 제공 개념

**한 줄 정의:** Service Client가 Server에 보내는 요청 데이터.

Service에서는 `Request`, Action에서는 개념적으로 `Goal`이라는 용어를 쓴다.
`goal_handle.request`는 Action Goal 데이터를 가리키는 rclpy 속성이다.

### Response

**출처:** ROS 2 Service 제공 개념

**한 줄 정의:** Service Server가 Client에 반환하는 한 번의 응답 데이터.

```python
def callback(request, response):
    response.ready = True
    return response
```

### Result

**출처:** ROS 2 Action 제공 개념. 구체적인 Result 필드는 프로젝트 정의

**한 줄 정의:** Action 작업 종료 시 Client에 반환하는 최종 업무 결과.

```python
result = InitializeNode.Result()
result.success = True
return result
```

Feedback은 중간 보고이고 Result는 최종 보고이다.

### ROS Clock

**출처:** ROS 2 제공

**한 줄 정의:** ROS 메시지의 기록 시각에 사용하는 노드 시계.

```python
self.get_clock().now().to_msg()
```

메시지 `header.stamp`에 사용한다. timeout 계산에는 보통 `time.monotonic_ns()`를 사용한다.

### ROS Graph

**출처:** ROS 2 제공 개념

**한 줄 정의:** 실행 중인 Node와 Topic·Service·Action 연결 전체.

```bash
ros2 node list
ros2 node info /inspection/master_node
ros2 topic list
ros2 service list
ros2 action list
```

### ROS 2 상태와 프로젝트 상태

**출처:** ROS 2 실행 개념 + 프로젝트 정의 FSM

```text
ROS Node 실행 여부
≠ Node Health 상태
≠ 전체 시스템 상태
≠ 제품 물리 상태
≠ 장비 실제 상태
```

| 상태 | 답하는 질문 | 예시 |
|---|---|---|
| Node Health | 노드 프로그램은 정상인가? | `READY`, `INIT_BLOCKED` |
| System State | 전체 공정은 어떤 단계인가? | `RUN_SYS`, `PAUSED` |
| Product State | 제품은 어디에 있는가? | `FLIPPING`, `SENSOR3_WAIT` |
| Equipment State | 장비는 무엇을 하는가? | `RUN_CONV`, `CAPTURE_HOLD` |

현재 프로젝트는 ROS 2 `LifecycleNode`가 아니라 자체 Health 상태와 FSM을 사용한다.

---

## S

### SystemState

**출처:** 프로젝트 정의 (`inspection_common.constants.SystemState`, `inspection_interfaces/msg/SystemState.msg`)

**한 줄 정의:** Master가 소유하는 전체 검사 시스템의 운전 단계.

```text
SystemState
├─ BOOT = 0
├─ INITIALIZING = 1
├─ READY = 2
├─ RUN_SYS = 3
├─ PAUSING = 4
├─ PAUSED = 5
├─ FAULT_STOP = 6
└─ RESETTING = 7
```

`RUN_SYS` 안에서도 촬영을 위해 컨베이어가 잠시 멈출 수 있다. 그 촬영 정지는 시스템 `PAUSED`가 아니다.
`PAUSING`은 정지 명령을 보냈지만 Control의 실제 정지 완료를 아직 확인하지 못한 전이 상태다.

### SystemEvent

**출처:** 프로젝트 정의 (`inspection_master.system_fsm.SystemEvent`)

**한 줄 정의:** 현재 `SystemState`를 다음 상태로 넘기도록 전이표에 입력하는 논리 이벤트.

```text
BOOT + APP_STARTED → INITIALIZING
INITIALIZING + INIT_DONE → READY
READY + START_REQUEST → READY          # 운전 명령만 요청
READY + ALL_CONVEYORS_RUNNING → RUN_SYS # 실제 RUN 확인
RUN_SYS + PAUSE_REQUEST → PAUSING
PAUSING + ALL_CONVEYORS_STOPPED → PAUSED
PAUSED + RESUME_REQUEST → PAUSED       # 재가동 명령만 요청
PAUSED + ALL_CONVEYORS_RUNNING → RUN_SYS
```

등록되지 않은 상태·이벤트 조합은 추정하지 않고 `InvalidSystemTransition`으로 거부한다.
초기화·시작·재개·Reset 실패는 안전 정지 상태에서 재시도하고, 정지 실패·추적 붕괴·E-stop은
`FAULT_STOP`으로 전환한다.

### StationResult

**출처:** 프로젝트 정의 (`inspection_interfaces/msg/StationResult.msg`)

**한 줄 정의:** Vision이 비동기 추론을 완료한 뒤 Master에 보내는 스테이션 단위 정상 결과.

```text
StationResult
├─ product_id, fifo_sequence, station_id
├─ capture_id, frame_batch_id, inference_job_id
├─ result_revision
├─ verdict: PASS 또는 NG
├─ score, model_version
└─ completed_at
```

`result_revision`이 더 큰 결과만 최신 결과로 반영한다. 같은 revision에 내용이 다르면 정합성 충돌이다.

### StationInferenceFailed

**출처:** 프로젝트 정의 (`inspection_interfaces/msg/StationInferenceFailed.msg`)

**한 줄 정의:** 촬영 이후 비동기 추론 작업이 명시적으로 실패했음을 Vision이 알리는 이벤트.

Master는 이 이벤트가 유효한 제품과 일치하면 해당 제품을 즉시 `FORCED_NG`로 잠근다.

### Server

**출처:** ROS 2 제공 개념

**한 줄 정의:** Service Request 또는 Action Goal을 받아 기능을 제공하는 쪽.

```text
Service Server → 짧은 요청에 Response 반환
Action Server  → Goal을 수행하고 Feedback·Result 반환
```

### Service

**출처:** ROS 2 제공

**한 줄 정의:** 짧은 Request·Response 통신.

```text
Service Client → Request → Service Server
Service Client ← Response ← Service Server
```

프로젝트 예: `GetNodeStatus`.

`.srv` 구조:

```text
Request 필드
---
Response 필드
```

### Session ID

**출처:** 프로젝트 정의

**한 줄 정의:** 재부팅·Reset 전후의 운전 실행 구간을 구분하는 프로젝트 ID.

```text
이전 실행 session-001
새 실행   session-002
```

이전 세션에서 늦게 도착한 메시지를 현재 세션 메시지로 오인하지 않도록 한다.

### spin

**출처:** ROS 2 제공 Executor 동작

**한 줄 정의:** Executor가 ROS 사건을 계속 기다리고 Callback을 처리하도록 실행하는 루프.

```python
executor.spin()
```

노드가 `spin()` 중이라는 것은 ROS 통신을 처리 중이라는 뜻이며 시스템이 `RUN_SYS`라는 뜻이 아니다.

### spin_node()

**출처:** 프로젝트 정의 (`inspection_common.node_base.spin_node`)

**한 줄 정의:** 공통 MultiThreadedExecutor 생성부터 안전 종료까지 묶어둔 실행 보조 함수.

```text
spin_node(node)
├─ MultiThreadedExecutor(num_threads=4) 생성
├─ executor.add_node(node)
├─ executor.spin()
├─ KeyboardInterrupt 처리
├─ executor.shutdown()
├─ node.destroy_node()
└─ rclpy.shutdown()
```

각 실행 노드의 `main()`은 직접 Executor를 중복 구현하지 않고 `spin_node(node)`를 호출한다.

### Subscriber·Subscription

**출처:** ROS 2 제공

**한 줄 정의:** Topic Message를 수신하고 연결된 Callback을 실행하는 구독 객체.

```python
self.create_subscription(
    MasterHeartbeat,
    "/inspection/master/heartbeat",
    self._handle_master_heartbeat,
    heartbeat_qos(),
)
```

메시지가 오면 ROS가 다음처럼 실행한다.

```python
self._handle_master_heartbeat(message)
```

### 상속

**출처:** Python 객체지향 문법

**한 줄 정의:** 부모 클래스의 기능을 자식 클래스가 물려받는 객체지향 구조.

```python
class InspectionNodeBase(Node):
    ...

class MasterNode(InspectionNodeBase):
    ...
```

```text
rclpy.node.Node
    ↓
InspectionNodeBase
    ↓
MasterNode·ControlNode·VisionNode·LogNode
```

`super().__init__()`은 부모 생성자를 호출한다.

---

## T

### Timer

**출처:** ROS 2 제공

**한 줄 정의:** 일정 주기 또는 시간 후 Callback을 실행하는 ROS 객체.

```python
self.create_timer(
    0.5,
    self._publish_heartbeat,
)
```

0.5초마다 Executor가 `_publish_heartbeat()`를 호출한다.

### Topic

**출처:** ROS 2 제공

**한 줄 정의:** Publisher가 비동기 단방향으로 Message를 방송하고 Subscriber가 수신하는 통신 방식.

```text
Publisher → Topic → Subscriber
```

프로젝트 예:

- Heartbeat
- 센서 이벤트
- 장비 상태
- 비전 결과
- 오류·로그 이벤트

Topic에는 직접적인 응답·완료 확인·취소 기능이 없다.

---

## V

### VisionQueueState

**출처:** 프로젝트 정의 (`inspection_interfaces/msg/VisionQueueState.msg`)

**한 줄 정의:** Vision 추론 Queue의 수용 가능 여부와 막힌 FrameBatch를 Master에 알리는 최신 상태 Message.

```text
VisionQueueState
├─ state: ACCEPTING / ENQUEUE_BLOCKED / DRAINING / FAULT
├─ depth, capacity
├─ blocked_frame_batch_id
└─ reason
```

`ENQUEUE_BLOCKED`이면 Master는 동일 FrameBatch를 보존한 채 `PAUSING`으로 전환한다.
Queue가 다시 `ACCEPTING`이 되고 실제 컨베이어 정지도 확인된 뒤 자동 재개할 수 있다.

---

## W

### Watchdog

**출처:** 일반 하드웨어·임베디드 시스템 개념

**한 줄 정의:** PC·프로그램·통신이 멈췄을 때 독립 장치가 안전 동작을 강제하는 감시 기능.

```text
ROS Heartbeat → 노드 간 소프트웨어 생존 감시
Mega Watchdog → PC·통신 고장 시 모터·출력 안전 확보
```

### Workspace

**출처:** ROS 2 개발 구조

**한 줄 정의:** 여러 ROS 2 Package를 함께 빌드하고 설치하는 작업공간.

```text
ros2_ws/
├─ src/
├─ build/
├─ install/
└─ log/
```

```bash
colcon build
source install/setup.bash
```

---

# 4. 프로젝트 ID 사전

| ID | 한 줄 정의 | 사용 목적 |
|---|---|---|
| `session_id` | 한 번의 운전 세션 ID | 이전 실행 메시지 차단 |
| `message_id` | 개별 메시지 ID | 메시지 추적·중복 확인 |
| `correlation_id` | 원본 요청 연결 ID | 요청과 응답·이벤트 연결 |
| `request_id` | `InitializeNode` 초기화 요청 ID | 초기화 재전송 멱등성 보장 |
| `retry_of_request_id` | 이전 초기화 요청 ID | 명시적인 새 재시도와 원본 연결 |
| `command_id` | 장비·시스템 명령 ID(UUIDv4) | 동작 명령 중복 실행 방지 |
| `command_epoch` | 명령 세대 번호 | Reset 전후 명령 구분 |
| `product_id` | 제품 ID | FIFO·결과·로그 연결 |
| `fifo_sequence` | 제품 유입 순번 | 비동기 결과를 물리 FIFO 순서로 복원 |
| `capture_id` | 한 번의 촬영 요청 ID | Master 요청과 Vision 작업 연결 |
| `frame_batch_id` | 실제 동기 촬영으로 얻은 이미지 묶음 ID | 여러 카메라 이미지와 재시도 결과 구분 |
| `inference_job_id` | 추론 Queue 작업 ID | 촬영 완료 후 비동기 추론 추적 |
| `sensor3_event_id` | Sensor3 물리 이벤트 ID | 최종 제품과 실제 분류 위치 연결 |
| `actuation_id` | 실제 액추에이터 실행 ID | 분류 동작 완료·중복 추적 |

```text
초기화: 같은 request_id + 같은 내용 → 기존 결과 반환
초기화: 같은 request_id + 다른 내용 → COMMAND_CONFLICT
업무 명령: 같은 command_id + 같은 payload_digest → 기존 실행 결과 재사용
업무 명령: 같은 command_id + 다른 payload_digest → COMMAND_CONFLICT
```

ROS Action 내부 Goal UUID와 프로젝트의 `request_id`는 별개이다.

v2 인터페이스에는 별도의 `image_id`가 없다. 개별 이미지는 `frame_batch_id` 안에서
`ImageReference.camera_id`, `file_path`, `sha256` 조합으로 식별하고 무결성을 확인한다.

---

# 5. 자주 헷갈리는 개념 비교

## `__init__()` vs `InitializeNode` vs `INITIALIZING`

```text
__init__()      → Python 객체와 ROS 통신 창구 생성
InitializeNode  → 실제 장비·모델·DB 준비 Action
INITIALIZING    → 전체 시스템 FSM 상태
```

## `Publisher 생성` vs `메시지 발행`

```python
create_publisher()  # 발행 창구 생성
publisher.publish() # 실제 메시지 발행
```

## `Callback 등록` vs `Callback 실행`

```python
self._publish_heartbeat   # 함수 자체를 등록
self._publish_heartbeat() # 지금 실행
```

## `Feedback` vs `Result`

```text
Feedback → 작업 중간에 여러 번 가능
Result   → 작업 종료 시 한 번
```

## `Goal ACCEPT` vs `작업 성공`

```text
GoalResponse.ACCEPT → 요청을 처리하겠음
result.success=True → 실제 업무 성공
goal_handle.succeed() → ROS Action 성공 종료
```

## `Heartbeat` vs `Watchdog`

```text
Heartbeat → ROS 노드 간 생존신호
Watchdog  → 통신·프로그램 사망 시 하드웨어 안전 동작
```

## `ROS Clock` vs `Monotonic Clock`

```text
ROS Clock       → 로그·메시지 발생 시각
Monotonic Clock → timeout·경과 시간 계산
```

## `Node Health` vs `System State`

```text
Node Health  → 개별 노드가 정상인가?
System State → 전체 공정이 어떤 운전 단계인가?
```

---

# 6. 코드 읽기 순서

```text
1. inspection_interfaces의 .msg·.srv·.action
2. inspection_common/constants.py
3. inspection_common/package_version.py
4. inspection_common/node_base.py
5. 각 노드의 __init__()와 Callback 등록
6. Master의 시스템 FSM·FIFO
7. Control·Vision·Log의 업무 로직
```

`node_base.py` 생성자는 다음 흐름으로 읽는다.

```text
ROS Node 생성
→ Parameter 선언
→ Callback Group 생성
→ Heartbeat Publisher 생성
→ Master Heartbeat Subscription 생성
→ 상태조회 Service 생성
→ 초기화 Action Server 생성
→ Timer 등록
→ spin_node()에서 Callback 처리
```

---

# 7. 설치 후 확인 명령 사전

| 확인 목적 | 명령 |
|---|---|
| 실행 노드 목록 | `ros2 node list` |
| 노드 통신 정보 | `ros2 node info /inspection/master_node` |
| Topic 목록과 타입 | `ros2 topic list -t` |
| Heartbeat 내용 | `ros2 topic echo /inspection/master/heartbeat` |
| Topic 연결·QoS | `ros2 topic info -v /inspection/master/heartbeat` |
| Service 목록 | `ros2 service list -t` |
| Action 목록 | `ros2 action list -t` |
| 노드 Parameter | `ros2 param list /inspection/master_node` |
| Parameter 값 | `ros2 param get /inspection/master_node profile` |
| Message 구조 | `ros2 interface show inspection_interfaces/msg/NodeHeartbeat` |
| Service 구조 | `ros2 interface show inspection_interfaces/srv/GetNodeStatus` |
| Action 구조 | `ros2 interface show inspection_interfaces/action/InitializeNode` |

---

# 8. 참고 자료

- [ROS 2 Jazzy: Topic·Service·Action 구분](https://docs.ros.org/en/jazzy/How-To-Guides/Topics-Services-Actions.html)
- [ROS 2 Jazzy: 인터페이스 정의](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Interfaces.html)
- [ROS 2: Executor와 Callback Group](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Executors.html)
- [ROS 2 Jazzy: Launch 파일](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Launch-system.html)
- [ROS 2 Jazzy: 입문 CLI 학습 목차](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools.html)

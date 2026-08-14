# inspection_common

여러 노드에서 동일하게 사용하는 최소 Python 코드만 두는 패키지입니다.

## 향후 포함 가능 항목

- 공통 ID 생성 보조 함수
- 설정 파일 로더
- 공통 QoS 생성 함수
- 여러 노드가 동일하게 사용하는 값 객체
- Heartbeat 발행과 상태조회 Service의 공통 기반 클래스
- 작업 노드의 초기화 Action 공통 골격

## 포함하지 않는 항목

- Master 전용 FIFO와 시스템 FSM
- Control 전용 모터·센서 상태
- Vision 전용 카메라·추론 큐
- Log 전용 SQLite 저장 로직

## 현재 제공 기능

`InspectionNodeBase`가 모든 노드에 다음 기능을 제공합니다.

- `/inspection/{node}/heartbeat` 주기 발행
- `/inspection/{node}/get_status` 상태조회 Service
- Control·Vision·Log의 `/inspection/{node}/initialize` Action
- Master 전용 `MasterHeartbeat`와 작업 노드 `NodeHeartbeat` 분리
- `Best Effort / Keep Last 1` Heartbeat QoS
- `MultiThreadedExecutor` 기반 공통 실행·종료 처리
- hardware 설정 검증 미구현 시 `INIT_BLOCKED` 처리

공통 기반 클래스는 통신 껍데기만 제공하며 각 노드의 실제 초기화 작업은 담당 패키지에서 구현합니다.

## 팀 개발 계약

모든 실행 노드는 `InspectionNodeBase`를 상속하고 생성자 첫 단계에서 자신의
`NodeId`와 초기화 Action 제공 여부를 부모 생성자에 전달합니다.

```python
class VisionNode(InspectionNodeBase):
    def __init__(self) -> None:
        super().__init__(NodeId.VISION, provides_initialize_action=True)
```

- Master는 `provides_initialize_action=False`, Control·Vision·Log는 `True`를 사용합니다.
- 공통 Heartbeat Publisher, `get_status` Service, 초기화 Action Server를 노드에서 중복 생성하지 않습니다.
- 공통 초기화 요청 검증과 결과 처리는 `_execute_initialize()`가 담당하므로 자식 노드에서 이 메서드를 오버라이딩하지 않습니다.
- 실제 장비·모델·저장소 초기화는 `initialize_node_resources()`만 오버라이딩하여 구현합니다.
- `hardware` 프로필을 사용하는 작업 노드는 `required_hardware_parameters()`도 오버라이딩하여 필수 설정 키를 반환합니다.
- 노드 Health 상태는 `set_health_state()`로 변경합니다. 전체 시스템 상태는 Master만 소유합니다.
- 실행 진입점은 `rclpy.init()` → 노드 객체 생성 → `spin_node(node)` 순서를 유지합니다.
- `inspection_interfaces` 메시지 필드와 공통 endpoint는 통합 합의 없이 개별 노드에서 변경하지 않습니다.

## 초기화 확장 지점

`InspectionNodeBase._execute_initialize()`는 멱등성, 인터페이스·설정 검증,
Action 결과 처리를 공통으로 수행한 뒤 다음 메서드를 호출합니다.

```python
async def initialize_node_resources(self) -> NodeInitializationOutcome:
    ...
```

담당자는 이 메서드 안에서만 자기 노드의 자원을 준비하고 결과를 반환합니다.

- Control: Mega 연결, 통신 확인, 안전 기본 출력
- Vision: 카메라 연결, 모델 로드, 추론 큐 준비
- Log: SQLite, 스키마, 이미지·spool 경로, 저장 용량

기본 구현은 `sim` 통신 골격만 성공시킵니다. `hardware`에서는 실제 구현이
완료되지 않은 노드를 `INIT_BLOCKED`로 유지합니다. 예외를 직접 삼키지 않고
실패 결과를 반환하거나 예외가 발생하면 공통 Action 처리에서 실패 응답으로 변환합니다.

## 변경 금지 경계

다음 항목은 노드 담당자가 자기 패키지 구현을 위해 수정할 영역이 아닙니다.

- Master 전용 FIFO와 시스템 FSM을 작업 노드에 복제
- 제품 ID 또는 최종 판정을 Control·Vision·Log에서 독립 생성
- 공통 Header·Heartbeat·상태조회 메시지 계약을 노드별로 변경
- `_execute_initialize()` 전체 복사 또는 재구현

공통 패키지나 인터페이스 변경이 필요하면 먼저 통합 담당자와 합의합니다.

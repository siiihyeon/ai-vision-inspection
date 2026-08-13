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

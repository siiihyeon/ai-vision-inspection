# inspection_master

`MasterNode` 패키지이며 사용자 담당입니다.

## 소유 책임

- 전체 시스템 상태
- 제품 ID와 단일 활성 FIFO
- 제품별 물리 진행 상태
- 스테이션 결과 반영과 최종 판정
- Control·Vision·Log 명령 조율
- 전체 `FAULT_STOP` 결정

## 소유하지 않는 것

- 센서 원신호와 모터 실제 상태
- 카메라 촬영·추론 내부 상태
- SQLite와 이미지 파일 저장 구현

## 현재 단계

`master_node` 실행 진입점과 공통 Heartbeat·상태조회 골격이 있습니다. Control·Vision·Log Heartbeat 구독과 초기화 Action Client도 생성합니다. Master는 초기화 요청의 주체이므로 자기 초기화 Action Server는 두지 않습니다.

다음 기능은 아직 없습니다.

- 시스템 FSM
- 제품 FIFO
- 초기화 Goal 전송 순서와 재시도
- 업무용 Topic·Service·Action

## NodeBase 호환 계약

- `MasterNode(InspectionNodeBase)` 상속을 유지합니다.
- 부모 생성자는 `NodeId.MASTER, provides_initialize_action=False`로 호출합니다.
- Master는 초기화 Action Server를 만들지 않고 Control·Vision·Log의 Action Client를 소유합니다.
- 공통 Master Heartbeat와 `get_status` Service를 중복 생성하지 않습니다.
- `system_state`, `command_epoch`, 제품 ID와 FIFO는 Master에서만 변경합니다.
- 작업 노드 Heartbeat 수신 시 노드 ID, 세션, sequence, 인터페이스 버전과 수신 monotonic 시각을 검증하는 로직을 구현해야 합니다.
- 실행 진입점의 `rclpy.init()` → `MasterNode()` → `spin_node(node)` 순서를 유지합니다.
- 공통 패키지나 `inspection_interfaces`를 변경해야 하면 다른 노드 담당자와 먼저 합의합니다.

Master는 초기화 요청의 주체이므로 `required_hardware_parameters()`와
`initialize_node_resources()`를 구현하지 않습니다.

## 실행

```bash
ros2 run inspection_master master_node
```

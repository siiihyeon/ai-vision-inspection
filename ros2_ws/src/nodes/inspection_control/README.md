# inspection_control

`ControlNode` 패키지이며 Control 담당자 작업 영역입니다.

## 소유 책임

- Arduino Mega 통신
- 센서 원신호·디바운스·물리 이벤트 순번
- 상층·하층 컨베이어 실제 상태와 스텝 위치
- 액추에이터 실제 route·작업·완료 상태
- TB6600 안전 출력과 물리 E-stop 보고
- 하드웨어 트리거와 조명 전기 출력

## 소유하지 않는 것

- 제품 ID와 물리 순서 FIFO
- 제품 최종 OK·NG 판정
- 카메라 프레임과 추론 결과
- 영구 로그의 원본

## 현재 단계

`control_node` 실행 진입점, Heartbeat, 상태조회 Service, 초기화 Action 골격이 있습니다. Mega와 실물 장비에 출력하지 않습니다.

## NodeBase 호환 계약

- `ControlNode(InspectionNodeBase)` 상속을 유지합니다.
- 부모 생성자는 `NodeId.CONTROL, provides_initialize_action=True`로 호출합니다.
- `_execute_initialize()`는 오버라이딩하지 않습니다.
- `required_hardware_parameters()`에 Mega·모터·센서·액추에이터의 필수 ROS 파라미터 키를 반환합니다.
- `initialize_node_resources()`에서 Mega 연결, 통신 확인, 안전 기본 출력과 장비 준비 상태를 검증합니다.
- Heartbeat Publisher, `get_status` Service와 초기화 Action Server를 중복 생성하지 않습니다.
- 노드 Health 상태는 `set_health_state()`로 변경하며, 전체 시스템 상태와 제품 FIFO를 직접 변경하지 않습니다.
- 실행 진입점의 `rclpy.init()` → `ControlNode()` → `spin_node(node)` 순서를 유지합니다.

현재 두 확장 메서드는 TODO 골격입니다. `sim`은 통신 시험을 허용하지만,
`hardware`는 필수 설정과 실제 초기화가 구현되기 전까지 `INIT_BLOCKED`가 정상입니다.

## 담당자 구현 체크리스트

- [ ] ROS 파라미터 선언과 `required_hardware_parameters()` 목록 일치
- [ ] Mega 연결·재연결과 heartbeat/watchdog 구현
- [ ] 센서 디바운스와 event sequence 구현
- [ ] 상·하층 모터 상태·스텝 위치 보고
- [ ] 액추에이터 route·완료 보고
- [ ] E-stop과 TB6600 안전 출력 검증
- [ ] 실패 시 `NodeInitializationOutcome`의 오류 코드·이유·재시도 가능 여부 반환

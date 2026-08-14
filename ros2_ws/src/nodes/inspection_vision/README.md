# inspection_vision

`VisionNode` 패키지이며 Vision 담당자 작업 영역입니다.

## 소유 책임

- 카메라 연결·설정·ARM·프레임 수집
- 제품×스테이션 촬영 작업
- 이미지 ID와 이미지 파일 생성
- 추론 작업 큐와 모델 실행
- 카메라별 점수와 스테이션 결과

## 소유하지 않는 것

- 제품 ID 생성과 물리 FIFO
- 컨베이어·액추에이터 직접 제어
- 제품 최종 판정 잠금
- 전체 `FAULT_STOP` 결정

## 현재 단계

`vision_node` 실행 진입점, Heartbeat, 상태조회 Service, 초기화 Action 골격이 있습니다. 카메라와 GPU를 사용하지 않습니다.

## NodeBase 호환 계약

- `VisionNode(InspectionNodeBase)` 상속을 유지합니다.
- 부모 생성자는 `NodeId.VISION, provides_initialize_action=True`로 호출합니다.
- `_execute_initialize()`는 오버라이딩하지 않습니다.
- `required_hardware_parameters()`에 카메라·조명·모델·GPU의 필수 ROS 파라미터 키를 반환합니다.
- `initialize_node_resources()`에서 카메라 연결, 모델 로드와 추론 워커·큐 준비를 검증합니다.
- Heartbeat Publisher, `get_status` Service와 초기화 Action Server를 중복 생성하지 않습니다.
- 노드 Health 상태는 `set_health_state()`로 변경하며, 제품 FIFO와 최종 판정을 직접 변경하지 않습니다.
- 실행 진입점의 `rclpy.init()` → `VisionNode()` → `spin_node(node)` 순서를 유지합니다.

현재 두 확장 메서드는 TODO 골격입니다. `sim`은 통신 시험을 허용하지만,
`hardware`는 필수 설정과 실제 초기화가 구현되기 전까지 `INIT_BLOCKED`가 정상입니다.

## 담당자 구현 체크리스트

- [ ] ROS 파라미터 선언과 `required_hardware_parameters()` 목록 일치
- [ ] 카메라별 연결·설정·ARM과 프레임 수집 구현
- [ ] `product_id`·`station_id`·`capture_id` 기준 촬영 멱등 처리
- [ ] 스테이션별 필수 이미지 수집 후 비동기 추론 작업 생성
- [ ] 모델·임계값 버전과 카메라별 점수 보고
- [ ] 촬영·추론 실패와 재시도 결과 보고
- [ ] 실패 시 `NodeInitializationOutcome`의 오류 코드·이유·재시도 가능 여부 반환

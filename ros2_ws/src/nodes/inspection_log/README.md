# inspection_log

`LogNode` 패키지이며 Log 담당자 작업 영역입니다.

## 소유 책임

- 제품·촬영·추론·시스템·오류 기록
- SQLite 원본
- 이미지 상대 경로와 제품 로그 연결
- 저장소 health와 spool 상태
- 과거 로그 조회

## 소유하지 않는 것

- 시스템 운전 상태 결정
- 제품 FIFO와 최종 판정 변경
- 장비·카메라 제어
- 운전 명령 중계

## 현재 단계

`log_node` 실행 진입점, Heartbeat, 상태조회 Service, 초기화 Action 골격이 있습니다. 파일이나 데이터베이스를 생성하지 않습니다.

## NodeBase 호환 계약

- `LogNode(InspectionNodeBase)` 상속을 유지합니다.
- 부모 생성자는 `NodeId.LOG, provides_initialize_action=True`로 호출합니다.
- `_execute_initialize()`는 오버라이딩하지 않습니다.
- `required_hardware_parameters()`에 DB·이미지·spool 경로와 저장 용량의 필수 ROS 파라미터 키를 반환합니다.
- `initialize_node_resources()`에서 SQLite 연결, 스키마, 저장 경로와 용량을 검증합니다.
- Heartbeat Publisher, `get_status` Service와 초기화 Action Server를 중복 생성하지 않습니다.
- 노드 Health 상태는 `set_health_state()`로 변경하며, 시스템 상태와 제품 판정을 직접 변경하지 않습니다.
- 실행 진입점의 `rclpy.init()` → `LogNode()` → `spin_node(node)` 순서를 유지합니다.

현재 두 확장 메서드는 TODO 골격입니다. `sim`은 통신 시험을 허용하지만,
`hardware`는 필수 설정과 실제 초기화가 구현되기 전까지 `INIT_BLOCKED`가 정상입니다.

## 담당자 구현 체크리스트

- [ ] ROS 파라미터 선언과 `required_hardware_parameters()` 목록 일치
- [ ] SQLite 연결·스키마 버전과 마이그레이션 구현
- [ ] 제품·촬영·추론·시스템·오류 기록 구현
- [ ] 이미지 상대 경로와 제품·Capture 연결
- [ ] 저장소 health, 경고·한계 용량과 spool 재전송 구현
- [ ] 중복 로그의 멱등 저장과 과거 로그 조회 구현
- [ ] 실패 시 `NodeInitializationOutcome`의 오류 코드·이유·재시도 가능 여부 반환

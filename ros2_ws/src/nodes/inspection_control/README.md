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

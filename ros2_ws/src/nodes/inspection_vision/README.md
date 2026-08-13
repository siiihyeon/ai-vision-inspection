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

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

## 실행

```bash
ros2 run inspection_master master_node
```

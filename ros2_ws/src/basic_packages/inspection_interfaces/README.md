# inspection_interfaces

네 실행 노드가 공유하는 ROS 2 통신 계약 패키지입니다.

현재는 네 노드의 기동·상태 확인에 필요한 최소 계약만 제공합니다.

| 인터페이스 | 종류 | 용도 |
|---|---|---|
| `CommonHeader` | Message | 세션·메시지·연관 요청 식별 |
| `MasterHeartbeat` | Message | Master 세션·명령 세대·시스템 상태 보고 |
| `NodeHeartbeat` | Message | 노드 생존·health·인터페이스 버전 보고 |
| `GetNodeStatus` | Service | 노드 현재 상태의 짧은 조회 |
| `InitializeNode` | Action | Control·Vision·Log 초기화와 진행 단계 보고 |

센서, 컨베이어, 촬영, 추론, 액추에이터 인터페이스는 각 기능을 구현할 때 설계 문서에서 추가합니다.

## 변경 규칙

- 특정 노드 담당자가 단독으로 필드를 변경하지 않습니다.
- 송신 노드와 수신 노드 담당자가 함께 영향을 검토합니다.
- 필드 삭제·이름 변경은 관련 코드와 같은 Pull Request 묶음으로 처리합니다.

## 설계 연결

- `CommonHeader`: 01 문서 공통 Payload 정의
- `MasterHeartbeat`: COM-03
- `NodeHeartbeat`: COM-04
- `GetNodeStatus`: COM-02
- `InitializeNode`: COM-01·COM-05·COM-06

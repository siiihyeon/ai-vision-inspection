# Service 계약 메모

`GetNodeStatus`는 짧고 side-effect 없는 진단 조회입니다. typed 필드가 자동화 판단의 기준이며 `status_json`은 노드별 추가 진단용입니다. 요청 session이 현재 session과 다르면 `ready=false`입니다.

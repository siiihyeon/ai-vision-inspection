# Service 계약 메모

`GetNodeStatus`는 짧고 side-effect 없는 진단 조회입니다. typed 필드가 자동화 판단의 기준이며 `status_json`은 노드별 추가 진단용입니다. 요청 session이 현재 session과 다르면 `ready=false`입니다.

`OperatorCommand`는 개발 단계의 CLI와 향후 HMI가 Master FSM에 진입하는
단일 통로입니다. 장비를 직접 제어하지 않고 Master의 guard를 거쳐
`INITIALIZE/START/PAUSE/RESUME/RESET/CONFIRM_LINE_CLEAR`를 요청합니다.
`request_id`를 비우면 Master가 UUID를 발급하고 응답에 돌려줍니다.

`ReplayStationResults`는 Log의 현재 session durable Vision terminal을 Master가 page 단위로 다시 읽는 복구 전용 Service입니다. `offset/max_results`와 `next_offset/has_more`를 사용합니다. 이전 Master session은 거부하며 report 복구 용도로도 사용하지 않습니다.

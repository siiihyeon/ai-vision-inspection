# launch 이후 운전 명령 빠른 안내

전제: `ros2_ws/tools/operator_command.sh`를 `op`라는 별명(alias)으로 씁니다
(`~/.bash_aliases`에 등록됨). `build_helper.md`의 터미널 2(명령용) 준비가
끝난 상태에서 사용합니다.

## 1. 명령 목록

| 입력 | 뜻 |
|---|---|
| `op init` | 4개 노드 초기화 |
| `op start` | 운전 시작 (컨베이어 RUN) |
| `op pause` | 일시정지 |
| `op resume` | 재개 |
| `op reset` | FAULT_STOP/복구 불가 PAUSED에서 벗어나기 시도 |
| `op line-clear` | "라인을 물리적으로 비웠다"는 확인만 기록 (상태 전이는 안 함) |

## 2. 기본 실행 순서

```bash
op init
op start
```

`init`이 실패하면(hardware.yaml 미설정 등) `start`는 의미 없으니, 로그에서
각 worker의 READY 확인부터 봅니다.

## 3. `reset`과 `line-clear`는 다른 명령입니다

- 일반 FAULT_STOP(제자리 복구 가능): `op reset`만으로 끝납니다.
- 제품 추적 정합성이 깨진 심각한 FAULT_STOP(LINE_CLEAR_REQUIRED): 실제로
  라인에서 제품을 다 치운 뒤 `op line-clear` → `op reset` 순서로 **같이**
  써야 합니다. `line-clear` 없이 `reset`만 치면 실패합니다.
- `op line-clear`는 `READY` 상태에서 `start` 걸기 전에 "라인이 비어있다"를
  미리 확인해두는 용도로 단독으로도 씁니다.

어떤 케이스인지는 스크립트가 판단해주지 않으므로, launch 터미널의 로그나
`ros2 topic echo /inspection/master/heartbeat`의 `system_state`를 보고
직접 판단합니다.

## 4. 옵션

```bash
op start "재시작 사유"          # reason 직접 지정 (기본값: "operator start via op")
OPERATOR_ID=teammate op start   # operator_id 지정 (기본값: $USER)
```

# 공동 작업 규칙

## 1. 기본 원칙

- `main` 브랜치에 직접 기능 코드를 작성하지 않습니다.
- 담당자는 자신의 기능 브랜치에서 작업하고 Pull Request로 병합합니다.
- 한 노드가 다른 노드의 원본 상태를 직접 변경하지 않습니다.
- 설계 문서의 상태명·이벤트명·필드명을 임의로 축약하거나 변경하지 않습니다.

## 2. 권장 브랜치

```text
feature/master-node
feature/control-node
feature/vision-node
feature/log-node
```

작은 기능은 다음처럼 하위 목적을 붙일 수 있습니다.

```text
feature/master-node/system-fsm
feature/control-node/sensor-debounce
feature/vision-node/camera-adapter
feature/log-node/sqlite-schema
```

## 3. 패키지 소유권

- Master 담당: `ros2_ws/src/nodes/inspection_master/`
- Control 담당: `ros2_ws/src/nodes/inspection_control/`
- Vision 담당: `ros2_ws/src/nodes/inspection_vision/`
- Log 담당: `ros2_ws/src/nodes/inspection_log/`

다음 공통 패키지는 변경 전에 관련 담당자와 협의합니다.

- `ros2_ws/src/basic_packages/inspection_interfaces/`
- `ros2_ws/src/basic_packages/inspection_common/`
- `ros2_ws/src/basic_packages/inspection_bringup/`

`nodes/`와 `basic_packages/`는 사람을 위한 분류 폴더입니다. `colcon`은 `ros2_ws/src` 아래의 `package.xml`을 재귀 탐색하므로 각 패키지는 그대로 빌드됩니다.

## 4. 인터페이스 변경 규칙

Topic·Service·Action을 추가하거나 변경할 때 Pull Request에 다음을 적습니다.

1. 관련 설계 문서와 메시지 ID
2. 변경 필드와 변경 이유
3. 영향을 받는 송신 노드와 수신 노드
4. 중복 수신·재전송·타임아웃 처리 변화

인터페이스를 사용하는 노드의 Pull Request가 준비되기 전에는 기존 필드를 삭제하거나 이름을 변경하지 않습니다.

## 5. 커밋 예시

```text
docs: add control node responsibility guide
feat(master): add system state definitions
feat(vision): add mock camera adapter
test(log): add SQLite idempotency test
fix(control): preserve sensor event sequence after reconnect
```

## 6. Pull Request 확인 항목

- 담당 패키지 밖의 변경이 필요한 이유가 설명되어 있는가
- 상태·데이터 소유권을 위반하지 않는가
- `TODO(HARDWARE_REQUIRED)`를 임의 수치로 확정하지 않았는가
- 새 설정값과 오류 코드가 설계 문서와 일치하는가
- 테스트 또는 수동 확인 방법이 적혀 있는가
- 인터페이스를 변경했다면 `inspection_interfaces/package.xml` 버전을 함께 올렸는가
- 긴 추론·시리얼 I/O·파일 작업을 ROS 콜백에서 직접 실행하지 않는가

## 7. 콜백과 긴 작업

- Heartbeat·상태조회 콜백은 빠르게 반환해야 합니다.
- 추론, 카메라 SDK 대기, 시리얼 블로킹 I/O, 대용량 저장은 별도 worker에 전달합니다.
- `MultiThreadedExecutor`는 콜백 분리를 돕지만 긴 작업 자체를 자동으로 안전하게 만들지는 않습니다.
- 공유 상태를 여러 callback group이나 worker에서 변경할 때는 해당 노드 담당자가 동기화 방법을 명시합니다.

# ROS 2 빌드·실행 빠른 안내

기준 작업공간: `~/ai-vision-inspection/ros2_ws`

## 1. 작업공간으로 이동
cd ~/ai-vision-inspection/ros2_ws

## 2. ROS 2 기본 환경 적용
source /opt/ros/jazzy/setup.bash

## 3. 전체 패키지 빌드
colcon build --symlink-install --cmake-force-configure \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3

## 4. 빌드 결과 환경 적용
source install/setup.bash

## 5. 시뮬레이션 실행
ros2 launch inspection_bringup inspection_system.launch.py profile:=sim

Master·Control·Vision·Log 노드를 sim 프로필로 함께 실행합니다.

## 6. 빌드부터 실행까지 한 번에 수행

cd ~/ai-vision-inspection/ros2_ws && \
source /opt/ros/jazzy/setup.bash && \
colcon build --symlink-install --cmake-force-configure --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 && \
source install/setup.bash && \
ros2 launch inspection_bringup inspection_system.launch.py profile:=sim

## 7. 명령용 두 번째 터미널 준비

cd ~/ai-vision-inspection/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

## 8. 실행 노드 확인

ros2 node list

`/inspection/master_node`, `control_node`, `vision_node`, `log_node`가 보이면 정상입니다.

## 9. 정적 계약 검사

```bash
python3 tools/verify_skeleton.py
python3 tools/test_domain_contracts.py
python3 tools/test_vision_algorithms.py
```

패키지 구조·인터페이스 계약과 ROS 비의존 핵심 로직 테스트를 실행합니다.

이 PC처럼 `/usr/local/bin/python3`와 ROS의 system Python이 함께 있으면
`colcon`이 잘못된 Python을 골라 `em` import에서 실패할 수 있습니다. 따라서
위 build 명령의 `/usr/bin/python3` 지정은 제거하지 않습니다.

## 10. 실행 종료

```text
Ctrl+C
```

launch를 실행한 터미널에서 눌러 네 노드의 안전 종료 절차를 시작합니다.

## 전체 package test

```bash
colcon test --python-testing pytest --return-code-on-test-failure \
  --event-handlers console_direct+
colcon test-result --verbose
```

## 언제 다시 빌드해야 하나

- 일반 Python 코드만 수정: `--symlink-install` 상태에서는 재빌드 없이 반영되는 경우가 많습니다.
- `msg`, `srv`, `action`, `setup.py`, `package.xml`, `CMakeLists.txt` 수정: 반드시 재빌드합니다.
- 새 Python 파일이나 패키지 추가: 재빌드하는 것이 안전합니다.
- 판단이 애매함: 실행 중인 노드를 `Ctrl+C`로 종료한 뒤 다시 빌드합니다.

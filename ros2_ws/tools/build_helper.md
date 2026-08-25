# ROS 2 빌드·실행 빠른 안내

기준 작업공간: `~/ai-vision-inspection/ros2_ws`

## 1. 공통

### 명령용 두 번째 터미널 준비

launch를 띄운 첫 번째 터미널은 그대로 두고, 운전 명령(`op`)이나 topic 확인은 새 터미널에서 합니다. `op`는 `ros2_ws/tools/operator_command.sh`를 가리키는 셸 alias이며 리포에 포함되어 있지 않으므로, 아래 블록이 없으면 최초 1회 자동으로 `~/.bash_aliases`에 등록합니다 (이미 등록돼 있으면 건너뜁니다).

```bash
cd ~/ai-vision-inspection/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
grep -qxF "alias op='~/ai-vision-inspection/ros2_ws/tools/operator_command.sh'" ~/.bash_aliases 2>/dev/null || \
  echo "alias op='~/ai-vision-inspection/ros2_ws/tools/operator_command.sh'" >> ~/.bash_aliases
source ~/.bash_aliases
```

이제 이 터미널에서 `op init`, `op line-clear`, `op start` 처럼 바로 쓸 수 있습니다.

### 실행 노드 확인

```bash
ros2 node list
```

`/inspection/master_node`, `control_node`, `vision_node`, `log_node`가 보이면 정상입니다.

### 모니터링용 터미널

`ros2 topic echo`는 실행하면 계속 그 자리에서 실시간 출력하며 멈추지 않으므로, 보고 싶은 topic마다 터미널을 따로 엽니다. 각 터미널도 터미널 2와 같은 준비(`cd` + `source` 두 줄)가 먼저 필요합니다.

**터미널 3** — 컨베이어 RUNNING/STOPPED, 센서 CLEAR, 액추에이터 안전 상태를 실시간으로 봅니다.

```bash
ros2 topic echo /inspection/control/equipment_state
```

**터미널 4** — 제품 하나가 A/B 촬영 다 끝나고 최종 PASS/NG 판정이 확정될 때마다 뜹니다.

```bash
ros2 topic echo /inspection/master/product_result_locked
```

**터미널 5 (sim에서 `PositionSettled`를 수동으로 찍어 넣는 경우만)** — 그 값이 Master까지 잘 들어가는지 확인합니다.

```bash
ros2 topic echo /inspection/control/position_settled
```

### 실행 종료

```text
Ctrl+C
```

launch를 실행한 터미널에서 눌러 네 노드의 안전 종료 절차를 시작합니다.

### 언제 다시 빌드해야 하나

- 일반 Python 코드만 수정: `--symlink-install` 상태에서는 재빌드 없이 반영되는 경우가 많습니다.
- `msg`, `srv`, `action`, `setup.py`, `package.xml`, `CMakeLists.txt` 수정: 반드시 재빌드합니다.
- 새 Python 파일이나 패키지 추가: 재빌드하는 것이 안전합니다.
- 판단이 애매함: 실행 중인 노드를 `Ctrl+C`로 종료한 뒤 다시 빌드합니다.

### 정적 검사·package test

코드 검증(빌드/실행 절차와는 별개)은 `tools/README.md`에 정리되어 있습니다.

이 PC처럼 `/usr/local/bin/python3`와 ROS의 system Python이 함께 있으면
`colcon`이 잘못된 Python을 골라 `em` import에서 실패할 수 있습니다 — 그래서
아래 build 명령에는 `-DPython3_EXECUTABLE=/usr/bin/python3` 지정이 들어가
있고, 제거하면 안 됩니다.

## 2. Hardware

빌드부터 환경 적용까지 한 번에:

```bash
cd ~/ai-vision-inspection/ros2_ws && \
source /opt/ros/jazzy/setup.bash && \
colcon build --symlink-install --cmake-force-configure \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 && \
source install/setup.bash
```

실행 (별도 명령, 첫 번째 터미널에 계속 떠 있음):

```bash
ros2 launch inspection_bringup inspection_system.launch.py profile:=hardware
```

## 3. Sim

빌드부터 환경 적용까지 한 번에:

```bash
cd ~/ai-vision-inspection/ros2_ws && \
source /opt/ros/jazzy/setup.bash && \
colcon build --symlink-install --cmake-force-configure \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 && \
source install/setup.bash
```

실행 (별도 명령, 첫 번째 터미널에 계속 떠 있음):

```bash
ros2 launch inspection_bringup inspection_system.launch.py profile:=sim
```

Master·Control·Vision·Log 노드를 sim 프로필로 함께 실행합니다.

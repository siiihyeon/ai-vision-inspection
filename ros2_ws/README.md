# ROS 2 워크스페이스

이 폴더는 Ubuntu 24.04 / ROS 2 Jazzy에서 `colcon`으로 빌드하는 워크스페이스입니다.

현재 통신 골격 단계에서는 네 노드 실행과 초기화·상태조회·Heartbeat 최소 계약을 확인합니다. 프로젝트의 1~3단계 정의는 저장소 루트 `README.md`에 있으며, 제품·센서·촬영·추론·액추에이터 업무 인터페이스는 각 기능 구현 단계에서 추가합니다.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch inspection_bringup inspection_system.launch.py profile:=sim
```

ROS 2가 없는 PC에서는 다음 정적 검사를 실행할 수 있습니다.

```bash
python3 tools/verify_skeleton.py
```

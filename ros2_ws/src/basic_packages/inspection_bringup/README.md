# inspection_bringup

네 노드를 같은 namespace와 profile로 실행합니다.

```bash
ros2 launch inspection_bringup inspection_system.launch.py profile:=sim
```

`sim.yaml`의 queue 16/worker 1은 개발 기본값이지 생산 승인값이 아닙니다. `hardware.yaml`의 빈 값과 0은 누락 표시이며 임의로 지우거나 우회하지 않습니다. 파일별 설정은 [config README](config/README.md), Vision의 전체 결정·실험·수정 위치는 [Vision Node 완성 결정표](../../nodes/inspection_vision/README_COMPLETION_CHECKLIST.md)를 따릅니다.

# inspection_bringup

네 실행 노드를 함께 실행하고 공통 설정을 전달할 패키지입니다.

현재 다음 파일을 제공합니다.

- `launch/inspection_system.launch.py`: 네 노드 일괄 실행
- `config/sim.yaml`: 장비 없이 통신 골격을 시험하는 프로필
- `config/hardware.yaml`: 실제 장비 설정이 확정되기 전 시작을 차단하는 프로필

## 실행

```bash
ros2 launch inspection_bringup inspection_system.launch.py profile:=sim
```

`hardware` 프로필은 각 작업 노드가 `required_hardware_parameters()`를 구현하고 필수 설정을 검증하기 전까지 `INIT_BLOCKED`가 되는 것이 정상입니다.

이 패키지는 여러 노드에 영향을 주므로 변경 전에 통합 담당자와 협의합니다.

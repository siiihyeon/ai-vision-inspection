# AI 비전검수 소프트웨어

2층·1층 컨베이어에서 제품의 앞뒷면을 검사하고, 최종 판정에 따라 제품을 분류하는 ROS 2 기반 소프트웨어 프로젝트입니다.

현재 저장소는 **3단계 협업 골격** 상태입니다. 패키지·빈 노드 실행 구조와 초기화·상태조회·Heartbeat 최소 계약, Launch와 `sim/hardware` 실행 프로필까지 있으며 상태머신, 제품 흐름, 센서·모터·카메라·추론·저장 기능은 아직 구현하지 않았습니다.

## 개발 단계 기준

| 단계 | 범위 | 현재 상태 |
|---|---|---|
| 1단계 | 7개 ROS 2 패키지와 담당 영역 구성 | 완료 |
| 2단계 | 공통 Message·Service·Action 계약 | 완료 |
| 3단계 | 네 노드 최소 통신, Launch, sim/hardware 프로필 | 완료 |
| 후속 단계 | FSM, FIFO, 장비, 촬영·추론, 영구 저장 | 미구현 |

## 실행 노드와 담당

| 실행 노드 | 패키지 | 핵심 책임 | 담당 |
|---|---|---|---|
| `master_node` | `inspection_master` | 전체 시스템 FSM, 제품 ID·FIFO, 제품 진행, 최종 판정 | 사용자 |
| `control_node` | `inspection_control` | Mega, 센서, 컨베이어, 액추에이터 실제 상태·제어 | 팀원 1 |
| `vision_node` | `inspection_vision` | 카메라 촬영, 이미지, 추론, 스테이션 결과 | 팀원 2 |
| `log_node` | `inspection_log` | SQLite, 이미지 연결 정보, 운전·오류 로그 | 팀원 3 |

## 지원 패키지

지원 패키지는 실행 노드가 아닙니다.

| 패키지 | 역할 | 변경 규칙 |
|---|---|---|
| `inspection_interfaces` | Topic·Service·Action 통신 계약 | 단독 변경 금지, 관련 노드 담당자와 협의 |
| `inspection_common` | 모든 노드가 공유하는 최소 Python 코드 | 노드 전용 상태·로직 추가 금지 |
| `inspection_bringup` | 전체 Launch와 공통 실행 설정 | 통합 담당자 검토 필요 |

## 폴더 구조

```text
SW/
├─ .github/                  Pull Request 템플릿과 정적 CI
├─ .gitignore               빌드·실행 데이터 제외 규칙
├─ 구현_전_상세설계/          설계 원본
├─ README_비전검수_워크플로우.pdf  전체 Workflow 도표
├─ ros2_ws/                  ROS 2 워크스페이스
│  ├─ tools/                ROS 없는 환경용 정적 검증
│  └─ src/
│     ├─ basic_packages/
│     │  ├─ inspection_interfaces/
│     │  ├─ inspection_common/
│     │  └─ inspection_bringup/
│     └─ nodes/
│        ├─ inspection_master/
│        ├─ inspection_control/
│        ├─ inspection_vision/
│        └─ inspection_log/
├─ CONTRIBUTING.md          공동 작업 규칙
└─ README.md                프로젝트 입구 문서
```

## 기준 환경

- Ubuntu 24.04
- ROS 2 Jazzy
- Python 3.12
- Arduino Mega + TB6600

현재 Windows 폴더는 설계·Git 작업에도 사용할 수 있지만 ROS 2 빌드와 실행은 Ubuntu 24.04/Jazzy 환경을 기준으로 합니다.

## 3단계 골격 확인 방법

Ubuntu 24.04/Jazzy에서 다음 명령을 사용합니다.

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch inspection_bringup inspection_system.launch.py profile:=sim
```

현재 단계에서는 네 노드의 시작 로그와 Heartbeat·상태조회·초기화 통신 골격만 동작하는 것이 정상입니다. 실제 장비와 검사 동작은 수행하지 않습니다.

## 설계 단일 원본

기능을 구현할 때는 `구현_전_상세설계/` 문서를 기준으로 합니다. 코드와 설계가 충돌하면 임의로 둘 중 하나를 바꾸지 않고 Pull Request에서 변경 이유와 영향 노드를 확인합니다.

전체 흐름을 빠르게 확인할 때는 [`README_비전검수_워크플로우.pdf`](README_비전검수_워크플로우.pdf)를 먼저 보고, 세부 계약은 `구현_전_상세설계/`의 원본 문서를 확인합니다.

`hardware` 프로필은 각 작업 노드가 필수 장비 설정 검증을 구현하기 전까지 초기화가 차단됩니다. 설정 검증을 우회하는 단일 수동 플래그는 사용하지 않습니다.

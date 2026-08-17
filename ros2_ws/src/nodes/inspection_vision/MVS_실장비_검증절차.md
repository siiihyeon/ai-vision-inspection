# HIKROBOT MVS 실장비 검증 절차

이 절차는 production PC의 native Ubuntu 24.04에서 수행합니다. WSL2는 ROS build와
sim 통신 시험용이며 GigE NIC timing, camera SDK, GPU 장시간 성능의 최종 합격
환경으로 사용하지 않습니다.

## 1. 시험 전 기록

- Ubuntu/kernel, ROS 2 Jazzy, Python 3.12 patch version
- MVS installer 파일명과 SHA-256, `MV_CC_GetSDKVersion()` 값 및 5.0.2 대응 근거
- 카메라 네 대의 model/serial/MAC/IP/firmware 실물 라벨과 enum 출력
- NIC model/driver/interface/IP/MTU/offload 설정
- GPU model/driver/CUDA/PyTorch version
- 시험 branch와 commit SHA, `hardware.yaml` SHA-256

## 2. 장치 identity 검증

1. 공정 네트워크와 분리된 camera 전용 NIC를 `192.168.10.10/24`로 설정합니다.
2. MVS 도구와 Python wrapper에서 GigE device를 enumerate합니다.
3. `CAM_A_1..3`, `CAM_B_1`의 serial, MAC, IP를 한 행씩 비교합니다.
4. 중복 serial/IP, 다른 model, firmware 불일치는 시험을 중단합니다.
5. 확정값을 `hardware.yaml`에 넣고 config snapshot/digest를 기록합니다.
6. vendor 대응표로 확인한 SDK raw 정수를 `vision.mvs.expected_sdk_version_raw`에 넣고,
   다른 SDK에서 초기화가 실패하는지 확인합니다.

카메라를 ID 배열 위치만 보고 연결하지 않습니다. serial이 camera ID의 주 identity이고
IP/MAC/model/firmware는 잘못 연결된 장치를 차단하는 추가 guard입니다.

## 3. SDK 기능 smoke test

카메라별로 다음 호출이 성공하는지 확인합니다.

- exclusive open/close/destroy 반복 100회
- optimal packet size 설정
- image node 8, OneByOne strategy
- PixelFormat BayerRG8, Width 2448, Height 2048
- Bayer conversion quality 1
- TriggerSelector FrameBurstStart, burst 1
- TriggerMode On, TriggerSource Action1, ActionSelector 1
- device/group/mask write/readback
- callback 등록/start/stop grabbing 반복
- `MV_CC_ClearImageBuffer`
- `MV_CC_ConvertPixelTypeEx` RGB8 packed 정확한 byte 길이

모든 SDK return code는 hex로 저장합니다. unsupported node를 조용히 무시하지 않습니다.
PTP node만 “미지원 가능” 항목이며 미지원 시 synchronized=false로 계속 시험할 수 있습니다.

## 4. Action group 격리 시험

1. Station A GroupKey 1로 1,000번 Action을 보냅니다.
2. 매번 A 카메라 3대는 각 1 frame, B 카메라는 0 frame인지 확인합니다.
3. Station B GroupKey 2도 반대로 1,000번 반복합니다.
4. 같은 카메라가 한 Action에 2 frame을 보내거나 다른 station이 반응하면 실패입니다.
5. ACK result count/address/status와 실제 callback frame 수를 함께 기록합니다.

scheduled Action은 사용하지 않으므로 `bActionTimeEnable=0`, `nActionTime=0`을 확인합니다.

## 5. 프레임 상관관계 시험

- `nTriggerIndex`가 Action마다 증가하는지 확인
- `nFrameNum` 증가/uint32 wrap 모의시험
- Action 직전 buffer에 의도적으로 old frame을 넣고 clear 뒤 연결되지 않는지 확인
- 첫 attempt frame을 지연시켜 두 번째 attempt에 섞이지 않는지 확인
- callback 진입 host monotonic이 trigger 요청보다 뒤인지 확인
- callback source pointer를 함수 반환 뒤 사용하지 않고 즉시 copy하는지 확인

`nTriggerIndex`가 항상 0이면 frame-number fallback을 사용한다는 사실과 제한을 시험
보고서에 명시합니다.

## 6. RGB channel golden test

순수 R/G/B와 ColorChecker를 촬영해 다음을 비교합니다.

1. camera BayerRG8 source
2. MVS RGB8 packed 변환 결과
3. 저장된 RGB PNG
4. OpenCV `imread` BGR 결과
5. `cvtColor(BGR2RGB)` 이후 model 입력

최종 model tensor의 R/G/B가 원 장면과 일치해야 합니다. 파일 viewer가 올바르게
보인다는 사실만으로 channel 계약을 승인하지 않습니다.

## 7. skew와 timeout 측정

다음 조건별로 최소 수천 batch를 수집합니다.

- idle / CPU·disk·GPU 동시 최대부하
- NIC 기본 MTU / 승인할 jumbo MTU 후보
- 최소/최대 exposure
- cold boot / 8시간 warm 상태
- Station A와 B 동시 요청

host callback arrival skew p50/p95/p99/p99.9/max와 frame timeout을 계산합니다.
허용 skew는 정상 p99.9보다 margin을 두되 잘못된 동기 촬영을 숨길 정도로 크게
잡지 않습니다. acquisition timeout은 Master Capture timeout 안에서 2 attempts와
PNG fsync 시간을 보장해야 합니다.

## 8. PTP 시험

1. GevIEEE1588 node 지원/쓰기 가능 여부를 camera별 기록합니다.
2. status가 master/slave/locked로 전이되는 실제 문자열 또는 enum을 기록합니다.
3. boot/reconnect마다 lock 도달시간과 flap을 측정합니다.
4. device raw tick 주파수와 ns 변환을 vendor 문서·실측으로 교차검증합니다.
5. 안정 유지시간을 확정하기 전 `camera_timestamp_synchronized=true`를 허용하지 않습니다.

PTP lock 실패가 immediate Action 자체를 막아서는 안 됩니다. 공식 capture skew는 host
arrival skew이며 PTP camera skew는 검증 telemetry입니다.

## 9. 장애 주입

- A1/A2/A3/B1 각각 cable 분리와 복구
- camera 전원 cycle
- NIC down/up
- packet loss/지연
- callback timeout
- PNG write 중 process kill
- Queue full 중 process kill
- inference 중 process kill
- disk warning/pause/critical threshold 도달
- LogNode 중단/복구
- ProductResultLocked와 worker 완료 동시 경합

각 경우 Master가 PAUSED/INIT_BLOCKED/FORCED_NG 중 승인된 상태로 가는지, 다른 제품의
identity가 섞이지 않는지, journal과 manifest가 어떤 상태로 남는지 확인합니다.

## 10. 합격 후 반영

1. `hardware.yaml`의 빈 값과 0을 실측값으로 변경합니다.
2. [실험_파라미터와_미결정사항.md](실험_파라미터와_미결정사항.md)의 해당 행을
   “확정”으로 바꾸고 시험 보고서 링크를 남깁니다.
3. config digest를 갱신합니다.
4. Python/정적/단위/ROS build·test와 full launch를 다시 실행합니다.
5. 별도 Git branch와 review를 거칩니다. 기존 shared branch에 force push하지 않습니다.

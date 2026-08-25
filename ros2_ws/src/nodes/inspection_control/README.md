# inspection_control

Control은 Mega, sensor 원신호, TB6600 실제 동작과 actuator 완료의 유일한 소유자입니다. 카메라 trigger와 LED를 제어하지 않습니다.

## 골격에 구현된 경계

- `PositionProduct`와 `ActuateProduct` Action server
- UUID/digest/epoch/session 검증과 멱등 결과 재생
- sim open-loop `PositionSettled`
- serial adapter가 호출할 `publish_sensor_observation()`
- `SystemCommand.target_conveyor_id`로 지정한 상·하층 개별 재가동 확장점
- `SystemCommand`의 전체 컨베이어 PAUSE/RESUME을 Mega `STOP`/`RUN`으로 전달하는 확장점 (RESET은 별도 Mega 명령 없음 - FAULT_STOP 진입 시 이미 STOP됨)
- hardware 필수 설정 누락과 adapter 미구현 시 READY 차단
- Mega의 `E|STATE` 이벤트를 `EquipmentState`로 옮기는 발행 경로 (변경 시에만 발행)

## 반드시 결정할 전장/프로토콜

| 항목 | 정확히 필요한 정보 |
|---|---|
| Mega link | `/dev/...`, baud, USB identity, reconnect/backoff, handshake/firmware version |
| packet | framing, command/event IDs, integer byte order, length, CRC, sequence, ACK/NACK, retry/timeout |
| watchdog | host/Mega heartbeat, timeout 시 Mega가 강제할 safe outputs, 재개 handshake |
| pin map | 모든 sensor, STEP/DIR/ENABLE, actuator output/input의 Mega pin과 active level |
| Sensor | Sensor1/2/3 debounce ms, rising/falling 의미, CLEAR/rearm, stuck 기준, event sequence persistence |
| TB6600 | microstep switch, motor/gear/pulley, steps/rev·mm, 방향, 최대속도, 가감속, 정지/settling |
| Position | station별 `target_step`, 허용 `position_error_steps`, open-loop 누적오차 복구/homing |
| Actuator | 명령별 동작/복귀 시간, feedback sensor, 완료/실패 조건, product passage 검증 |
| E-stop | 배선/입력, software 보고, reset 권한과 물리 재가동 절차 |

## 구현 인수 조건

- serial I/O는 ROS callback을 block하지 않고 전용 thread/queue를 사용합니다.
- 같은 command ID는 물리 출력을 두 번 발생시키지 않습니다.
- reconnect 후 오래된 epoch 명령을 실행하지 않습니다.
- Position·Actuation의 장시간 hardware 실행 루프는
  `goal_handle.is_cancel_requested`를 주기적으로 확인하고 취소 시 물리 출력을
  안전 상태로 만든 뒤 `goal_handle.canceled()`로 종료합니다.
- RESET 또는 `command_epoch` 변경은 진행 중인 이전 명령을 무효화해야 하며,
  Action cancel 수락만으로 물리 정지를 확정하지 않습니다.
- 장애 시 STEP/ENABLE/actuator가 문서화된 safe state가 됩니다.
- firmware protocol simulator와 hardware-in-loop test를 각각 둡니다.

Arduino placeholder는 저장소 `firmware/arduino_mega/README.md`에 있습니다.

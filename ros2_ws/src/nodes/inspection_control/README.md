# inspection_control

## Sensor3 진단 로그

Mega firmware의 CRC-framed
`LOG|SENSOR3|<event>|millis=...|micros=...|distanceCm=...|armed=...|detectCount=...|releaseCount=...`
메시지는 Control Node가 검증한 뒤 `MEGA_SENSOR3_<event>` DEBUG LogEvent로 변환합니다.
Log Node가 실행 중이면 원본 표본은 SQLite `log_events`에 저장되며 payload에는
`distance_cm`, `detection_armed`, detect/release 연속 횟수, release 판정,
재무장 및 echo timeout 여부가 포함됩니다. `READ`는 Sensor3의 모든 유효 측정을
기록하므로 장기 운영 전에는 데이터 증가량과 retention 정책을 별도로 확정해야 합니다.

CRC/ASCII/필드 형식 검증에서 탈락한 실제 serial 입력은 이유별로 모두 카운트합니다.
동일 원인의 경고와 `MEGA_SERIAL_FRAME_REJECTED` SQLite 이벤트는 최대 5초에 한 번만
기록하며, payload에 전체 누적 카운터, 입력 길이 및 최대 240자의 안전한 원문 미리보기를
포함합니다. serial timeout으로 생기는 빈 read는 오류 카운터에서 제외합니다.

Control은 Mega, sensor 원신호, TB6600 실제 동작과 actuator 완료의 유일한 소유자입니다. 카메라 trigger와 LED를 제어하지 않습니다.

## 골격에 구현된 경계

- `ActuateProduct` Action server (Position은 Action이 아닙니다 - 아래 참고)
- UUID/digest/epoch/session 검증과 멱등 결과 재생
- serial adapter가 호출할 `publish_sensor_observation()`
- `SystemCommand.target_conveyor_id`로 지정한 상·하층 개별 재가동 확장점
- `SystemCommand`의 전체 컨베이어 PAUSE/RESUME을 Mega `STOP`/`RUN`으로 전달하는 확장점 (RESET은 별도 Mega 명령 없음 - FAULT_STOP 진입 시 이미 STOP됨)
- hardware 필수 설정 누락과 adapter 미구현 시 READY 차단
- Mega의 `E|STATE` 이벤트를 `EquipmentState`로 옮기는 발행 경로 (변경 시에만 발행)
- Station 개별 재가동(`handle_targeted_conveyor_command`)이 보낸 `RUN`의 Mega
  ACK을 확인하면 `ConveyorResumed`를 1회성으로 발행 (`_handle_run_ack` ->
  `_publish_conveyor_resumed`). `EquipmentState`는 depth가 낮은 상태
  스냅샷이라 같은 시각 상태가 연달아 바뀌면 중간 RUNNING 전이가 구독자에
  아예 전달되지 않을 수 있는데, 이 이벤트는 Mega의 확정 ACK을 근거로 하므로
  그 유실 경로와 무관합니다. 전체 컨베이어 RESUME(`handle_all_conveyors_command`)에는
  적용하지 않습니다 — 그쪽 확인(`confirm_all_conveyors_running`)은 이미
  level 체크라 이 문제에 노출되지 않습니다.

Position은 Master가 명령하지 않습니다. Mega가 센서 감지 후 `SET_OFFSET`으로
받아둔 step 수만큼 자율로 이동·정지하고, `E|POSITION`을 그대로
`PositionSettled`로 옮겨 발행합니다(`_publish_position_settled`). sim
profile에는 이 자율 이동을 대신 흉내낼 컴포넌트가 없으므로,
`SensorEvent`처럼 `PositionSettled`도 시험 중 `ros2 topic pub`으로 직접
발행해야 합니다.

## 반드시 결정할 전장/프로토콜

| 항목 | 정확히 필요한 정보 |
|---|---|
| Mega link | `/dev/...`, baud, USB identity, reconnect/backoff, handshake/firmware version |
| packet | framing, command/event IDs, integer byte order, length, CRC, sequence, ACK/NACK, retry/timeout |
| watchdog | host/Mega heartbeat, timeout 시 Mega가 강제할 safe outputs, 재개 handshake |
| pin map | 모든 sensor, STEP/DIR/ENABLE, actuator output/input의 Mega pin과 active level |
| Sensor | Sensor1/2/3 debounce ms, rising/falling 의미, CLEAR/rearm, stuck 기준, event sequence persistence |
| TB6600 | microstep switch, motor/gear/pulley, steps/rev·mm, 방향, 최대속도, 가감속, 정지/settling |
| Position | station별 `SET_OFFSET` step 수, open-loop 누적오차 복구/homing |
| Actuator | 명령별 동작/복귀 시간, feedback sensor, 완료/실패 조건, product passage 검증 |
| E-stop | 배선/입력, software 보고, reset 권한과 물리 재가동 절차 |

## 구현 인수 조건

- serial I/O는 ROS callback을 block하지 않고 전용 thread/queue를 사용합니다.
- 같은 command ID는 물리 출력을 두 번 발생시키지 않습니다.
- reconnect 후 오래된 epoch 명령을 실행하지 않습니다.
- Actuation의 장시간 hardware 실행 루프는
  `goal_handle.is_cancel_requested`를 주기적으로 확인하고 취소 시 물리 출력을
  안전 상태로 만든 뒤 `goal_handle.canceled()`로 종료합니다. Position은
  Action이 아니므로 취소 대상이 아니며, 정지는 `SystemCommand` PAUSE(→Mega
  `STOP`)로만 이뤄집니다.
- RESET 또는 `command_epoch` 변경은 진행 중인 이전 명령을 무효화해야 하며,
  Action cancel 수락만으로 물리 정지를 확정하지 않습니다.
- 장애 시 STEP/ENABLE/actuator가 문서화된 safe state가 됩니다.
- firmware protocol simulator와 hardware-in-loop test를 각각 둡니다.

Arduino placeholder는 저장소 `firmware/arduino_mega/README.md`에 있습니다.

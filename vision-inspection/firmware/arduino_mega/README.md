# Arduino Mega firmware placeholder

사용자 승인에 따라 폴더만 포함합니다. 현재 실행 가능한 firmware는 없습니다.

## 코드를 만들기 전에 반드시 결정할 사항

- Mega board/USB identity와 baud
- packet framing, command/event ID, byte order, payload length
- CRC 종류/초기값, sequence, ACK/NACK, retry/timeout
- host와 Mega watchdog 및 timeout safe state
- Sensor1/2/3 pin, active level, debounce, rearm, stuck
- 상·하 TB6600 STEP/DIR/ENABLE pin, microstep, calibration, speed/acceleration
- actuator pin, active level, pulse/hold/return timing과 feedback
- E-stop/limit/home wiring과 reset procedure
- firmware semantic version과 host compatibility handshake

## 금지사항

- LED 밝기/ON/OFF 기능을 넣지 않습니다. 조명은 외부 LED controller 상시점등입니다.
- camera hardware/global trigger pulse를 생성하지 않습니다. Trigger는 VisionNode의 MVS GigE Action Command입니다.
- protocol/핀맵 미확정 상태에서 임의 번호를 production default로 넣지 않습니다.

최종 구현 시 firmware source, protocol specification, pin-map, simulator와 HIL test 절차를 이 폴더에 함께 추가합니다.

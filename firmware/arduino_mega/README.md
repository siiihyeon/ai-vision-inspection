# Arduino Mega firmware

`arduino_mega.ino`는 ControlNode와 CRC-16 줄 기반으로 통신하며 센서 이벤트,
컨베이어 RUN/STOP/POSITION, NG 서보 분류를 수행합니다. 상세 패킷은
`PROTOCOL.md`에 있습니다.

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

`arduino_mega.ino` 상단의 핀맵은 승인된 배선도 값으로 반드시 채워야 합니다.
현재 `255`로 두어 미확정 배선에서 입력과 출력이 비활성화됩니다.

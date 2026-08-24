# Mega serial protocol

Frames are ASCII lines terminated by `\n`. The final field is an uppercase CRC-16/CCITT-FALSE of every byte before the final `|`.

Host commands:

- `C|sequence|HELLO|protocol_version|crc`
- `C|sequence|RUN|1-or-2|crc` and `STOP`
- `C|sequence|POSITION|1-or-2|step_count|crc`
- `C|sequence|ACTUATE|1(NG)-or-2(PASS)|crc`

Mega replies with `A|sequence|OK-or-ERR|crc`. Events are `E|SENSOR|SENSOR_1..3|1|sensor_sequence|estimated_step|crc`, `E|POSITION|conveyor|step_count|position_command_sequence|crc`, `E|ACTUATION|OK|actuation_command_sequence|crc`, and `E|STATE|upper_state|lower_state|sensor_1_clear|sensor_2_clear|sensor_3_clear|actuator_safe|crc`.

`E|STATE` reports equipment safety status and is sent only when something
changes, not on a fixed period. `upper_state`/`lower_state` use the same
numbering as the Mega's internal conveyor state machine (0=running,
1=positioning, 2=waiting at camera, 3=stopped). `sensor_1_clear`
through `sensor_3_clear` and `actuator_safe` are `0`/`1`.

Sensor 1 and 2 only report a debounced rising edge. Master then requests `POSITION`; the corresponding conveyor stops after the requested step count and echoes the command sequence in the position event. Sensor 3 reports an edge without stopping its conveyor. `ACTUATE=1` briefly rotates the NG servo; `ACTUATE=2` is a pass-through no-op.
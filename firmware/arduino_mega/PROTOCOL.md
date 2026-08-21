# Mega serial protocol

Frames are ASCII lines terminated by `\n`. The final field is an uppercase CRC-16/CCITT-FALSE of every byte before the final `|`.

Host commands:

- `C|sequence|HELLO|protocol_version|crc`
- `C|sequence|RUN|1-or-2|crc` and `STOP`
- `C|sequence|POSITION|1-or-2|step_count|crc`
- `C|sequence|ACTUATE|1(NG)-or-2(PASS)|crc`

Mega replies with `A|sequence|OK-or-ERR|crc`. Events are `E|SENSOR|SENSOR_1..3|1|sensor_sequence|estimated_step|crc`, `E|POSITION|conveyor|step_count|crc`, and `E|ACTUATION|OK|crc`.

Sensor 1 and 2 only report a debounced rising edge. Master then requests `POSITION`; the corresponding conveyor stops after the configured step count. Sensor 3 reports an edge without stopping its conveyor. `ACTUATE=1` briefly rotates the NG servo; `ACTUATE=2` is a pass-through no-op.
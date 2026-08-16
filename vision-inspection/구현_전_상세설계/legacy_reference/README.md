# Legacy reference

이 폴더의 PDF는 수정 레퍼런스의 물리 흐름과 상태머신 도식을 추적하기 위해 보존한 과거 자료입니다.

PDF에 있는 다음 내용은 폐기되었으며 구현에 사용하지 않습니다.

- ControlNode/Arduino Mega의 camera hardware/global trigger
- ROS 또는 Arduino의 LED 점등·밝기·안정화 제어
- `HARDWARE_LINE` trigger mode

현재 계약은 VisionNode의 HIKROBOT MVS `GIGE_ACTION_COMMAND` broadcast와 외부 LED controller 상시점등 방식입니다. 정확한 구현 기준은 상위 `README.md`, `노드별_책임과_상태_소유권.md`, 현재 ROS interface 파일입니다.

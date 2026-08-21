#include <Arduino.h>
#include <Servo.h>

// Fill these values from the approved wiring diagram before installing hardware.
// 255 deliberately disables the output/input instead of guessing a production pin.
const uint8_t SENSOR_1_PIN = 255;
const uint8_t SENSOR_2_PIN = 255;
const uint8_t SENSOR_3_PIN = 255;
const uint8_t UPPER_STEP_PIN = 255;
const uint8_t UPPER_DIR_PIN = 255;
const uint8_t UPPER_ENABLE_PIN = 255;
const uint8_t LOWER_STEP_PIN = 255;
const uint8_t LOWER_DIR_PIN = 255;
const uint8_t LOWER_ENABLE_PIN = 255;
const uint8_t SERVO_PIN = 255;

const uint32_t SENSOR_DEBOUNCE_MS = 40;
const uint32_t STEP_PERIOD_US = 1000;
const uint16_t SERVO_NG_ANGLE = 90;
const uint16_t SERVO_PASS_ANGLE = 0;

uint16_t crc16(const char *text) {
  uint16_t crc = 0xFFFF;
  while (*text) {
    crc ^= static_cast<uint16_t>(*text++) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit)
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
  }
  return crc;
}

void sendFrame(const char *body) {
  Serial.print(body);
  Serial.print('|');
  char crc[5];
  snprintf(crc, sizeof(crc), "%04X", crc16(body));
  Serial.println(crc);
}

struct Conveyor {
  uint8_t stepPin;
  uint8_t dirPin;
  uint8_t enablePin;
  bool running;
  bool positioning;
  int32_t remainingSteps;
  uint32_t lastStepUs;
};

Conveyor conveyors[2] = {{UPPER_STEP_PIN, UPPER_DIR_PIN, UPPER_ENABLE_PIN, false, false, 0, 0},
                         {LOWER_STEP_PIN, LOWER_DIR_PIN, LOWER_ENABLE_PIN, false, false, 0, 0}};
Servo sorter;

struct SensorState { uint8_t pin; bool stable; bool lastRaw; uint32_t changedAt; uint32_t sequence; };
SensorState sensors[3] = {{SENSOR_1_PIN, false, false, 0, 0}, {SENSOR_2_PIN, false, false, 0, 0}, {SENSOR_3_PIN, false, false, 0, 0}};

void acknowledge(const char *sequence, const char *status) {
  char body[48];
  snprintf(body, sizeof(body), "A|%s|%s", sequence, status);
  sendFrame(body);
}

void setConveyor(uint8_t index, bool run) {
  if (index > 1 || conveyors[index].enablePin == 255) return;
  conveyors[index].running = run;
  conveyors[index].positioning = false;
  digitalWrite(conveyors[index].enablePin, run ? LOW : HIGH); // TB6600 enable is commonly active-low.
}

void startPosition(uint8_t index, int32_t steps) {
  if (index > 1 || conveyors[index].stepPin == 255 || steps <= 0) return;
  Conveyor &conveyor = conveyors[index];
  conveyor.remainingSteps = steps;
  conveyor.positioning = true;
  conveyor.running = true;
  digitalWrite(conveyor.enablePin, LOW);
  digitalWrite(conveyor.dirPin, HIGH);
}

void tickConveyor(Conveyor &conveyor) {
  if (!conveyor.running || conveyor.stepPin == 255 ||
      (conveyor.positioning && conveyor.remainingSteps <= 0)) {
    if (conveyor.positioning) {
      conveyor.positioning = false;
      conveyor.running = false;
      if (conveyor.enablePin != 255) digitalWrite(conveyor.enablePin, HIGH);
      const uint8_t index = (&conveyor == &conveyors[0]) ? 1 : 2;
      char body[64];
      snprintf(body, sizeof(body), "E|POSITION|%u|0", index);
      sendFrame(body);
    }
    return;
  }
  if (micros() - conveyor.lastStepUs < STEP_PERIOD_US) return;
  conveyor.lastStepUs = micros();
  digitalWrite(conveyor.stepPin, HIGH);
  delayMicroseconds(4);
  digitalWrite(conveyor.stepPin, LOW);
  if (conveyor.positioning) --conveyor.remainingSteps;
}

void emitSensor(uint8_t index) {
  char body[96];
  const uint8_t edge = 1; // SensorEvent.RISING
  snprintf(body, sizeof(body), "E|SENSOR|SENSOR_%u|%u|%lu|0", index + 1, edge,
           static_cast<unsigned long>(sensors[index].sequence));
  sendFrame(body);
}

void pollSensor(uint8_t index) {
  SensorState &sensor = sensors[index];
  if (sensor.pin == 255) return;
  const bool raw = digitalRead(sensor.pin) == HIGH;
  if (raw != sensor.lastRaw) { sensor.lastRaw = raw; sensor.changedAt = millis(); }
  if (millis() - sensor.changedAt < SENSOR_DEBOUNCE_MS || raw == sensor.stable) return;
  const bool rising = raw;
  sensor.stable = raw;
  if (rising) { ++sensor.sequence; emitSensor(index); }
}

void handleCommand(char *line) {
  char *fields[8] = {};
  uint8_t count = 0;
  char *token = strtok(line, "|");
  while (token && count < 8) { fields[count++] = token; token = strtok(nullptr, "|"); }
  if (count < 5 || strcmp(fields[0], "C") != 0) return;
  const uint16_t supplied = static_cast<uint16_t>(strtoul(fields[count - 1], nullptr, 16));
  line[strlen(line) - strlen(fields[count - 1]) - 1] = '\0';
  if (crc16(line) != supplied) return;
  const char *sequence = fields[1];
  const char *operation = fields[2];
  if (!strcmp(operation, "HELLO")) { acknowledge(sequence, "OK"); return; }
  if (!strcmp(operation, "RUN") || !strcmp(operation, "STOP")) {
    const uint8_t index = atoi(fields[3]) - 1;
    setConveyor(index, !strcmp(operation, "RUN")); acknowledge(sequence, "OK"); return;
  }
  if (!strcmp(operation, "POSITION")) {
    const uint8_t index = atoi(fields[3]) - 1;
    startPosition(index, atol(fields[4])); acknowledge(sequence, "OK"); return;
  }
  if (!strcmp(operation, "ACTUATE")) {
    if (atoi(fields[3]) == 1 && SERVO_PIN != 255) { sorter.write(SERVO_NG_ANGLE); delay(300); sorter.write(SERVO_PASS_ANGLE); }
    acknowledge(sequence, "OK"); sendFrame("E|ACTUATION|OK");
  }
}

void setup() {
  Serial.begin(115200);
  for (SensorState &sensor : sensors) if (sensor.pin != 255) pinMode(sensor.pin, INPUT);
  for (Conveyor &conveyor : conveyors) if (conveyor.enablePin != 255) { pinMode(conveyor.stepPin, OUTPUT); pinMode(conveyor.dirPin, OUTPUT); pinMode(conveyor.enablePin, OUTPUT); digitalWrite(conveyor.enablePin, HIGH); }
  if (SERVO_PIN != 255) { sorter.attach(SERVO_PIN); sorter.write(SERVO_PASS_ANGLE); }
}

void loop() {
  static char line[128]; static uint8_t length = 0;
  while (Serial.available()) { const char value = Serial.read(); if (value == '\n') { line[length] = '\0'; handleCommand(line); length = 0; } else if (length + 1 < sizeof(line)) line[length++] = value; }
  pollSensor(0); pollSensor(1); pollSensor(2); tickConveyor(conveyors[0]); tickConveyor(conveyors[1]);
}
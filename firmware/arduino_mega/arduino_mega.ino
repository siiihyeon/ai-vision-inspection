#include <AccelStepper.h>
#include <Servo.h>

// ============================================================
// AI Vision Inspection - Arduino Mega Firmware
//
// Workflow
// 1) HC-SR04 #1 detects product
//    -> report SENSOR_1
//    -> Conveyor 1 autonomously moves its configured camera offset
//       (configured by Control Node with SET_OFFSET)
//    -> Conveyor 1 moves to the camera position and stops
//    -> report POSITION settled
//
// 2) Master finishes capture
//    -> Control Node sends RUN|1
//    -> Conveyor 1 resumes
//
// 3) HC-SR04 #2 does the same thing for Conveyor 2.
//
// 4) HC-SR04 #3 only reports product arrival.
//    -> Conveyor 2 does NOT stop.
//
// 5) Control Node sends ACTUATE|1 for defective product.
//    -> angle-controlled MG996R moves 0 deg -> 70 deg -> waits -> 0 deg.
//    ACTUATE|2 = normal/pass, no servo motion.
//
// Serial framing is compatible with the existing CRC protocol:
//   C|seq|COMMAND|...|CRC
//   A|seq|OK|CRC
//   E|...|CRC
// ============================================================


// ============================================================
// 1. Pin map
//    Conveyor and Sensor 1 / Servo pins are copied from the
//    component-test sketches supplied by the project.
// ============================================================

// Conveyor 1 - tested pin map
#define CONV1_STEP 10
#define CONV1_DIR   9
#define CONV1_EN    8

// Conveyor 2 - tested pin map
#define CONV2_STEP  5
#define CONV2_DIR   4
#define CONV2_EN    3

// HC-SR04 #1 - tested pin map
#define TRIG1 25
#define ECHO1 24

// HC-SR04 #2
// TODO: change these two values if your actual wiring is different.
#define TRIG2 31
#define ECHO2 30

// HC-SR04 #3
// TODO: change these two values if your actual wiring is different.
#define TRIG3 45
#define ECHO3 40

// MG996R - tested pin map
#define SERVO_PIN 52


// ============================================================
// 2. Conveyor parameters
// ============================================================

AccelStepper conveyor1(AccelStepper::DRIVER, CONV1_STEP, CONV1_DIR);
AccelStepper conveyor2(AccelStepper::DRIVER, CONV2_STEP, CONV2_DIR);

// Values confirmed in the component-test sketch.
const long CONV1_SPEED = -4000;
const long CONV2_SPEED = 4000;

const long CONV_MAX_SPEED = 10000;
const long CONV_ACCELERATION = 10000;


// ============================================================
// 3. Ultrasonic parameters
// ============================================================

// Detection / release thresholds copied from servo_ultra.ino.
const float DETECT_DISTANCE_CM  = 10.0f;
const float RELEASE_DISTANCE_CM = 12.0f;

// Ping one sensor every 20 ms.
// With 3 sensors, each individual sensor is measured about every 60 ms.
// This also reduces ultrasonic crosstalk compared with triggering all
// three sensors at once.
const unsigned long GLOBAL_PING_INTERVAL_US = 10000UL;

// Same timeout used in the test sketches.
const unsigned long ECHO_TIMEOUT_US = 5000UL;

// Reject obviously invalid readings.
const float MIN_VALID_DISTANCE_CM = 1.5f;
const float MAX_VALID_DISTANCE_CM = 80.0f;

// A detection is confirmed only after this many consecutive valid
// measurements at or below DETECT_DISTANCE_CM.
const uint8_t REQUIRED_CONSECUTIVE_DETECTIONS = 5;

// A previously detected product is considered gone, and the sensor becomes
// armed again, only after this many consecutive no-object readings.
// A no-echo timeout and a valid reading beyond RELEASE_DISTANCE_CM both count
// as a no-object reading.
const uint8_t REQUIRED_CONSECUTIVE_RELEASES = 10;

// Sensor 3 raw echo telemetry is useful during bench diagnosis, but emitting
// it for every ping can delay safety-critical sensor events on the same serial
// link. Keep it off during normal inspection runs.
const bool SENSOR3_DIAGNOSTIC_LOG_ENABLED = false;


// ============================================================
// 4. Servo parameters - angle-controlled MG996R
//    Values copied from the successful angle-control test.
// ============================================================

Servo sorterServo;

const int SERVO_HOME_ANGLE = 0;
const int SERVO_WORK_ANGLE = 70;

// Time allowed for the servo to physically reach each angle.
const unsigned long SERVO_MOVE_DELAY_MS = 500UL;

// Sensor 3 stores Conveyor 2's position when the product arrives.  For an
// NG product, the arm returns home only after this many later TB6600 pulses.
const long SERVO_RETURN_AFTER_STEPS = 12000L;


// ============================================================
// 5. Conveyor state machines
// ============================================================

enum ConveyorState : uint8_t {
  CONV_RUNNING = 0,
  CONV_POSITIONING,
  CONV_WAIT_CAMERA,
  CONV_STOPPED
};

struct ConveyorController {
  AccelStepper* motor;
  uint8_t enablePin;
  long runSpeed;
  uint8_t conveyorId;
  ConveyorState state;
  // Sensor sequence that caused this autonomous positioning cycle.
  long positionSensorSequence;
  long positionTargetSteps;
  long cameraOffsetSteps;
};

ConveyorController conveyors[2] = {
  { &conveyor1, CONV1_EN, CONV1_SPEED, 1, CONV_STOPPED, 0, 0, 0 },
  { &conveyor2, CONV2_EN, CONV2_SPEED, 2, CONV_STOPPED, 0, 0, 0 }
};


// ============================================================
// 6. Ultrasonic state machine
// ============================================================

struct UltrasonicSensor {
  uint8_t trigPin;
  uint8_t echoPin;
  uint8_t sensorId;

  bool detectionArmed;
  uint32_t detectionSequence;
  float lastDistanceCm;
  uint8_t consecutiveDetectCount;
  uint8_t consecutiveReleaseCount;
};

UltrasonicSensor sensors[3] = {
  { TRIG1, ECHO1, 1, true, 0, -1.0f, 0, 0 },
  { TRIG2, ECHO2, 2, true, 0, -1.0f, 0, 0 },
  { TRIG3, ECHO3, 3, true, 0, -1.0f, 0, 0 }
};

// One product can wait at each upstream sensor while its conveyor is busy
// positioning the preceding product. The stored value is the remaining
// travel from the newly detected product to that conveyor's camera position.
bool pendingSensor1Detection = false;
long pendingSensor1RemainingSteps = 0;
long pendingSensor1Sequence = 0;
bool pendingSensor2Detection = false;
long pendingSensor2RemainingSteps = 0;
long pendingSensor2Sequence = 0;

enum SonicState : uint8_t {
  SONIC_IDLE = 0,
  WAIT_ECHO_HIGH,
  WAIT_ECHO_LOW
};

SonicState sonicState = SONIC_IDLE;

uint8_t activeSensorIndex = 0;
uint8_t nextSensorIndex = 0;

unsigned long lastGlobalPingUs = 0;
unsigned long pingStartUs = 0;
unsigned long echoStartUs = 0;


// ============================================================
// 7. Servo state machine
// ============================================================

enum ServoState : uint8_t {
  SERVO_READY = 0,
  SERVO_MOVING_TO_WORK,
  SERVO_WAITING,
  SERVO_MOVING_HOME
};

ServoState servoState = SERVO_READY;
unsigned long servoStateStartMs = 0;

// Captured at the confirmed Sensor 3 edge.  This is a software count of the
// STEP pulses issued by AccelStepper, not a physical encoder measurement.
long sensor3Conveyor2StartPosition = 0;
bool sensor3StepReferenceValid = false;

// Tracks the equipment state last reported to the host, so
// updateEquipmentStateReport() only sends E|STATE when something
// actually changed instead of on a fixed period.
bool equipmentStateReported = false;
int lastReportedUpperState = -1;
int lastReportedLowerState = -1;
bool lastReportedSensor1Clear = false;
bool lastReportedSensor2Clear = false;
bool lastReportedSensor3Clear = false;
bool lastReportedServoReady = false;

// startRejectCycle() runs non-blocking, so the originating ACTUATE
// command's sequence must be cached here to echo it back once the
// reject cycle actually completes in updateServo().
long pendingActuationSequence = 0;

// Control Node performs HELLO before a new ROS run.  The first RUN after that
// handshake must initialise the sorter and start both conveyors.  Later RUN
// commands resume only the requested conveyor after camera positioning.
bool rosStartupPending = true;


// ============================================================
// 8. Serial protocol
// ============================================================

const uint16_t PROTOCOL_VERSION = 2;
const unsigned long SERIAL_BAUD = 115200UL;

char rxBuffer[160];
size_t rxLength = 0;


// ============================================================
// CRC-16/CCITT-FALSE
// poly=0x1021, init=0xFFFF
// ============================================================

uint16_t crc16Ccitt(const char* text) {
  uint16_t crc = 0xFFFF;

  while (*text) {
    crc ^= ((uint16_t)(uint8_t)(*text++)) << 8;

    for (uint8_t i = 0; i < 8; ++i) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc <<= 1;
      }
    }
  }

  return crc;
}


void sendFrame(const char* body) {
  uint16_t crc = crc16Ccitt(body);

  Serial.print(body);
  Serial.print('|');

  char crcText[5];
  snprintf(crcText, sizeof(crcText), "%04X", crc);
  Serial.println(crcText);
}


void acknowledge(long sequence, const char* status) {
  char body[64];
  snprintf(body, sizeof(body), "A|%ld|%s", sequence, status);
  sendFrame(body);
}


void sendSensorEvent(
  uint8_t sensorId,
  uint32_t sensorSequence,
  long estimatedStep
) {
  // Format retained from the existing Control Node protocol:
  // E|SENSOR|SENSOR_n|edge|sensor_sequence|estimated_step
  char body[96];
  snprintf(
    body,
    sizeof(body),
    "E|SENSOR|SENSOR_%u|1|%lu|%ld",
    sensorId,
    (unsigned long)sensorSequence,
    estimatedStep
  );
  sendFrame(body);
}


void sendPositionSettled(uint8_t conveyorId, long movedSteps) {
  // E|POSITION|conveyor_id|step_count|sensor_sequence
  char body[96];
  ConveyorController& conveyor = conveyors[conveyorId - 1];
  snprintf(
    body,
    sizeof(body),
    "E|POSITION|%u|%ld|%ld",
    conveyorId,
    movedSteps,
    conveyor.positionSensorSequence
  );
  sendFrame(body);
}


void sendActuationEvent(const char* status, long sequence) {
  char body[64];
  snprintf(body, sizeof(body), "E|ACTUATION|%s|%ld", status, sequence);
  sendFrame(body);
}


void sendEquipmentState() {
  // E|STATE|upper_state|lower_state|s1_clear|s2_clear|s3_clear|servo_ready
  // Conveyor states use the same numbering as enum ConveyorState above.
  char body[96];
  snprintf(
    body,
    sizeof(body),
    "E|STATE|%d|%d|%d|%d|%d|%d",
    (int)conveyors[0].state,
    (int)conveyors[1].state,
    sensors[0].detectionArmed ? 1 : 0,
    sensors[1].detectionArmed ? 1 : 0,
    sensors[2].detectionArmed ? 1 : 0,
    servoState == SERVO_READY ? 1 : 0
  );
  sendFrame(body);
}


void logSensor3Event(const char* event, float distanceCm, bool armed) {
  if (!SENSOR3_DIAGNOSTIC_LOG_ENABLED) {
    return;
  }

  UltrasonicSensor& sensor = sensors[2];
  char body[180];

  snprintf(
    body,
    sizeof(body),
    "LOG|SENSOR3|%s|millis=%lu|micros=%lu|distanceCm=%.2f|armed=%d|detectCount=%u|releaseCount=%u",
    event,
    (unsigned long)millis(),
    (unsigned long)micros(),
    distanceCm,
    armed ? 1 : 0,
    (unsigned)sensor.consecutiveDetectCount,
    (unsigned)sensor.consecutiveReleaseCount
  );

  sendFrame(body);
}


void updateEquipmentStateReport() {
  int upperState = (int)conveyors[0].state;
  int lowerState = (int)conveyors[1].state;
  // A sensor is clear only after its two-sample release check completes.
  bool sensor1Clear = sensors[0].detectionArmed;
  bool sensor2Clear = sensors[1].detectionArmed;
  bool sensor3Clear = sensors[2].detectionArmed;
  bool servoReady = servoState == SERVO_READY;

  bool changed =
    !equipmentStateReported ||
    upperState != lastReportedUpperState ||
    lowerState != lastReportedLowerState ||
    sensor1Clear != lastReportedSensor1Clear ||
    sensor2Clear != lastReportedSensor2Clear ||
    sensor3Clear != lastReportedSensor3Clear ||
    servoReady != lastReportedServoReady;

  if (!changed) {
    return;
  }

  sendEquipmentState();

  equipmentStateReported = true;
  lastReportedUpperState = upperState;
  lastReportedLowerState = lowerState;
  lastReportedSensor1Clear = sensor1Clear;
  lastReportedSensor2Clear = sensor2Clear;
  lastReportedSensor3Clear = sensor3Clear;
  lastReportedServoReady = servoReady;
}


// ============================================================
// 9. Conveyor control
// ============================================================

void enableConveyor(ConveyorController& conveyor) {
  // TB6600 enable polarity copied from the test code.
  digitalWrite(conveyor.enablePin, LOW);
}


void startContinuousRun(uint8_t index) {
  if (index > 1) return;

  ConveyorController& conveyor = conveyors[index];

  enableConveyor(conveyor);
  conveyor.motor->setSpeed(conveyor.runSpeed);
  conveyor.state = CONV_RUNNING;

  // Service one detection that arrived while this conveyor was positioning
  // the preceding product. Processing it here prevents a valid Sensor 1/2
  // edge from being lost simply because the conveyor was temporarily busy.
  bool hasPendingDetection =
    (index == 0) ? pendingSensor1Detection : pendingSensor2Detection;

  if (hasPendingDetection) {
    long remainingSteps =
      (index == 0) ? pendingSensor1RemainingSteps : pendingSensor2RemainingSteps;
    long sensorSequence =
      (index == 0) ? pendingSensor1Sequence : pendingSensor2Sequence;

    // The sensor event was already emitted at the physical detection instant.
    // Clear the slot before starting the saved positioning move so a later
    // product can occupy it while this product is being processed.
    if (index == 0) {
      pendingSensor1Detection = false;
      pendingSensor1RemainingSteps = 0;
      pendingSensor1Sequence = 0;
    } else {
      pendingSensor2Detection = false;
      pendingSensor2RemainingSteps = 0;
      pendingSensor2Sequence = 0;
    }

    if (conveyor.cameraOffsetSteps > 0) {
      startAutomaticPosition(index, remainingSteps, sensorSequence);
    }
  }
}


// Run once at the beginning of each ROS session. This is deliberately not
// called by normal post-camera RUN commands, because those must not interrupt
// an in-progress reject cycle or restart the other conveyor.
void startRosSession() {
  sorterServo.write(SERVO_HOME_ANGLE);
  servoState = SERVO_READY;
  servoStateStartMs = millis();
  pendingActuationSequence = 0;
  sensor3StepReferenceValid = false;

  startContinuousRun(0);
  startContinuousRun(1);
  rosStartupPending = false;
}


void stopConveyor(uint8_t index) {
  if (index > 1) return;

  conveyors[index].state = CONV_STOPPED;
  // Motor stays enabled so the belt can hold position.
}


bool startAutomaticPosition(uint8_t index, long targetSteps, long sensorSequence) {
  if (index > 1) return false;

  ConveyorController& conveyor = conveyors[index];

  // Only a normally running conveyor can begin autonomous positioning.
  if (conveyor.state != CONV_RUNNING) {
    return false;
  }

  enableConveyor(conveyor);

  conveyor.motor->setCurrentPosition(0);

  // Both tested conveyor speeds are negative, so move in the same
  // physical direction by using a negative relative target.
  long signedOffset =
    (conveyor.runSpeed < 0) ? -labs(targetSteps) : labs(targetSteps);

  conveyor.motor->move(signedOffset);
  conveyor.motor->setMaxSpeed(labs(conveyor.runSpeed));
  conveyor.motor->setAcceleration(CONV_ACCELERATION);
  conveyor.positionSensorSequence = sensorSequence;
  conveyor.positionTargetSteps = targetSteps;

  conveyor.state = CONV_POSITIONING;
  return true;
}


void updateConveyor(uint8_t index) {
  if (index > 1) return;

  ConveyorController& conveyor = conveyors[index];

  switch (conveyor.state) {
    case CONV_RUNNING:
      conveyor.motor->runSpeed();
      break;

    case CONV_POSITIONING:
      conveyor.motor->run();

      if (conveyor.motor->distanceToGo() == 0) {
        conveyor.state = CONV_WAIT_CAMERA;

        // The host command sequence correlates this event with one product.
        sendPositionSettled(
          conveyor.conveyorId,
          conveyor.positionTargetSteps
        );
      }
      break;

    case CONV_WAIT_CAMERA:
      // Hold position until Master -> Control -> RUN command arrives.
      break;

    case CONV_STOPPED:
      break;
  }
}


// ============================================================
// 10. Product detection handling
// ============================================================

void handleDetection(uint8_t sensorIndex, float distanceCm) {
  if (sensorIndex > 2) return;

  UltrasonicSensor& sensor = sensors[sensorIndex];

  if (sensor.sensorId == 3) {
    logSensor3Event("READ", distanceCm, sensor.detectionArmed);
  }

  // A valid far-distance echo also counts as one no-object reading.
  // Do not re-arm after only one such reading: the previous product may still
  // be in the sensor area or the reading may be noisy.
  if (distanceCm >= RELEASE_DISTANCE_CM) {
    if (sensor.sensorId == 3) {
      logSensor3Event("RELEASE_CHECK", distanceCm, sensor.detectionArmed);
    }

    sensor.consecutiveDetectCount = 0;
    if (!sensor.detectionArmed &&
        sensor.consecutiveReleaseCount < REQUIRED_CONSECUTIVE_RELEASES) {
      ++sensor.consecutiveReleaseCount;
      if (sensor.consecutiveReleaseCount >= REQUIRED_CONSECUTIVE_RELEASES) {
        sensor.detectionArmed = true;
        sensor.consecutiveReleaseCount = 0;
        if (sensor.sensorId == 3) {
          logSensor3Event("REARMED", distanceCm, sensor.detectionArmed);
        }
      }
    }
    return;
  }

  // A value outside the detect zone breaks a consecutive-detection streak.
  if (distanceCm > DETECT_DISTANCE_CM) {
    sensor.consecutiveDetectCount = 0;
    // This is neither close enough to detect nor far enough to release.
    sensor.consecutiveReleaseCount = 0;
    return;
  }

  // Any close echo means the previous product has not left yet.
  sensor.consecutiveReleaseCount = 0;

  if (!sensor.detectionArmed) {
    return;
  }

  ++sensor.consecutiveDetectCount;

  if (sensor.consecutiveDetectCount < REQUIRED_CONSECUTIVE_DETECTIONS) {
    return;
  }

  // The required consecutive close measurements have now been confirmed.
  sensor.consecutiveDetectCount = 0;

  // Sensor 1 is associated with Conveyor 1.
  if (sensor.sensorId == 1) {
    if (conveyors[0].state != CONV_RUNNING) {
      // Only POSITIONING/WAIT_CAMERA are a genuine mid-cycle busy period with
      // a valid distanceToGo() reading. CONV_STOPPED (paused) has no active
      // move() target, so distanceToGo() would be stale garbage there -
      // silently drop the detection instead of queuing a bogus position.
      if (
        conveyors[0].state != CONV_POSITIONING &&
        conveyors[0].state != CONV_WAIT_CAMERA
      ) {
        return;
      }

      // Keep one busy-period detection instead of discarding it. Disarm this
      // sensor so repeated readings of the same product cannot overwrite the
      // pending product's remaining travel distance.
      if (!pendingSensor1Detection) {
        sensor.detectionArmed = false;
        sensor.consecutiveReleaseCount = 0;
        ++sensor.detectionSequence;
        long detectedStep = conveyors[0].motor->currentPosition();
        long currentRemaining = labs(conveyors[0].motor->distanceToGo());
        long pendingRemaining =
          conveyors[0].cameraOffsetSteps - currentRemaining;

        pendingSensor1RemainingSteps = constrain(
          pendingRemaining,
          0,
          conveyors[0].cameraOffsetSteps
        );
        pendingSensor1Sequence = sensor.detectionSequence;
        pendingSensor1Detection = true;
        sendSensorEvent(
          sensor.sensorId,
          sensor.detectionSequence,
          detectedStep
        );
      }
      return;
    }

    sensor.detectionArmed = false;
    sensor.consecutiveReleaseCount = 0;
    ++sensor.detectionSequence;

    // Report the edge before beginning the local positioning cycle.
    sendSensorEvent(
      sensor.sensorId,
      sensor.detectionSequence,
      conveyors[0].motor->currentPosition()
    );
    if (conveyors[0].cameraOffsetSteps > 0) {
      startAutomaticPosition(
        0,
        conveyors[0].cameraOffsetSteps,
        sensor.detectionSequence
      );
    }
    return;
  }

  // Sensor 2 is associated with Conveyor 2.
  if (sensor.sensorId == 2) {
    if (conveyors[1].state != CONV_RUNNING) {
      // Same CONV_STOPPED exclusion as Sensor 1 above.
      if (
        conveyors[1].state != CONV_POSITIONING &&
        conveyors[1].state != CONV_WAIT_CAMERA
      ) {
        return;
      }

      // Symmetric one-slot pending buffer for Conveyor 2 / Sensor 2.
      if (!pendingSensor2Detection) {
        sensor.detectionArmed = false;
        sensor.consecutiveReleaseCount = 0;
        ++sensor.detectionSequence;
        long detectedStep = conveyors[1].motor->currentPosition();
        long currentRemaining = labs(conveyors[1].motor->distanceToGo());
        long pendingRemaining =
          conveyors[1].cameraOffsetSteps - currentRemaining;

        pendingSensor2RemainingSteps = constrain(
          pendingRemaining,
          0,
          conveyors[1].cameraOffsetSteps
        );
        pendingSensor2Sequence = sensor.detectionSequence;
        pendingSensor2Detection = true;
        sendSensorEvent(
          sensor.sensorId,
          sensor.detectionSequence,
          detectedStep
        );
      }
      return;
    }

    sensor.detectionArmed = false;
    sensor.consecutiveReleaseCount = 0;
    ++sensor.detectionSequence;

    sendSensorEvent(
      sensor.sensorId,
      sensor.detectionSequence,
      conveyors[1].motor->currentPosition()
    );
    if (conveyors[1].cameraOffsetSteps > 0) {
      startAutomaticPosition(
        1,
        conveyors[1].cameraOffsetSteps,
        sensor.detectionSequence
      );
    }
    return;
  }

  // Sensor 3 only reports arrival.
  // It never changes Conveyor 2 state.
  if (sensor.sensorId == 3) {
    sensor.detectionArmed = false;
    sensor.consecutiveReleaseCount = 0;
    ++sensor.detectionSequence;

    logSensor3Event("RELEASED", distanceCm, sensor.detectionArmed);
    sensor3Conveyor2StartPosition = conveyors[1].motor->currentPosition();
    sensor3StepReferenceValid = true;
    sendSensorEvent(
      sensor.sensorId,
      sensor.detectionSequence,
      conveyors[1].motor->currentPosition()
    );
  }
}


// ============================================================
// 11. Non-blocking HC-SR04 scheduler
// ============================================================

void startPing(uint8_t sensorIndex) {
  UltrasonicSensor& sensor = sensors[sensorIndex];

  digitalWrite(sensor.trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(sensor.trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(sensor.trigPin, LOW);

  activeSensorIndex = sensorIndex;
  pingStartUs = micros();
  sonicState = WAIT_ECHO_HIGH;
}


void finishPing(float distanceCm) {
  UltrasonicSensor& sensor = sensors[activeSensorIndex];
  sensor.lastDistanceCm = distanceCm;

  if (
    distanceCm >= MIN_VALID_DISTANCE_CM &&
    distanceCm <= MAX_VALID_DISTANCE_CM
  ) {
    handleDetection(activeSensorIndex, distanceCm);
  } else {
    // An invalid value must not count as one of the three consecutive reads.
    sensor.consecutiveDetectCount = 0;
    sensor.consecutiveReleaseCount = 0;
  }

  lastGlobalPingUs = micros();
  nextSensorIndex = (activeSensorIndex + 1) % 3;
  sonicState = SONIC_IDLE;
}


void abortPing() {
  UltrasonicSensor& sensor = sensors[activeSensorIndex];

  // No echo is one no-object reading, not an immediate release.  A sensor
  // that has already detected a product is re-armed only after two
  // consecutive timeouts (or valid far-distance readings in handleDetection).
  sensor.lastDistanceCm = -1.0f;
  sensor.consecutiveDetectCount = 0;

  if (sensor.sensorId == 3) {
    logSensor3Event("TIMEOUT", -1.0f, sensor.detectionArmed);
  }

  if (!sensor.detectionArmed &&
      sensor.consecutiveReleaseCount < REQUIRED_CONSECUTIVE_RELEASES) {
    ++sensor.consecutiveReleaseCount;
    if (sensor.consecutiveReleaseCount >= REQUIRED_CONSECUTIVE_RELEASES) {
      sensor.detectionArmed = true;
      sensor.consecutiveReleaseCount = 0;
      if (sensor.sensorId == 3) {
        logSensor3Event("REARMED_TIMEOUT", -1.0f, sensor.detectionArmed);
      }
    }
  }

  lastGlobalPingUs = micros();
  nextSensorIndex = (activeSensorIndex + 1) % 3;
  sonicState = SONIC_IDLE;
}


void updateUltrasonicSensors() {
  unsigned long now = micros();
  UltrasonicSensor& sensor = sensors[activeSensorIndex];

  switch (sonicState) {
    case SONIC_IDLE:
      if (now - lastGlobalPingUs >= GLOBAL_PING_INTERVAL_US) {
        startPing(nextSensorIndex);
      }
      break;

    case WAIT_ECHO_HIGH:
      if (digitalRead(sensor.echoPin) == HIGH) {
        echoStartUs = micros();
        sonicState = WAIT_ECHO_LOW;
      }
      else if (now - pingStartUs > ECHO_TIMEOUT_US) {
        abortPing();
      }
      break;

    case WAIT_ECHO_LOW:
      if (digitalRead(sensor.echoPin) == LOW) {
        unsigned long durationUs = micros() - echoStartUs;
        float distanceCm = durationUs * 0.0343f / 2.0f;
        finishPing(distanceCm);
      }
      else if (now - echoStartUs > ECHO_TIMEOUT_US) {
        abortPing();
      }
      break;
  }
}


// ============================================================
// 12. Non-blocking MG996R reject cycle
// ============================================================

bool startRejectCycle(long sequence) {
  // An NG command must correspond to a product that already reached Sensor 3.
  if (servoState != SERVO_READY || !sensor3StepReferenceValid) {
    return false;
  }

  // Move from the home angle to the reject/work angle.
  sorterServo.write(SERVO_WORK_ANGLE);
  servoState = SERVO_MOVING_TO_WORK;
  servoStateStartMs = millis();
  pendingActuationSequence = sequence;

  return true;
}


void updateServo() {
  unsigned long now = millis();

  switch (servoState) {
    case SERVO_READY:
      break;

    case SERVO_MOVING_TO_WORK:
      // Servo.write() only commands the angle, so give the motor
      // enough time to physically reach SERVO_WORK_ANGLE.
      if (now - servoStateStartMs >= SERVO_MOVE_DELAY_MS) {
        servoState = SERVO_WAITING;
        servoStateStartMs = now;
      }
      break;

    case SERVO_WAITING:
      // Return based on Conveyor 2 travel measured from Sensor 3, not on a
      // fixed dwell time. This keeps the arm out until this product passes.
      if (
        labs(
          conveyors[1].motor->currentPosition() -
          sensor3Conveyor2StartPosition
        ) >= SERVO_RETURN_AFTER_STEPS
      ) {
        sorterServo.write(SERVO_HOME_ANGLE);
        servoState = SERVO_MOVING_HOME;
        servoStateStartMs = now;
      }
      break;

    case SERVO_MOVING_HOME:
      // After the arm has physically returned to 0 degrees,
      // report completion to the Control Node.
      if (now - servoStateStartMs >= SERVO_MOVE_DELAY_MS) {
        servoState = SERVO_READY;
        sendActuationEvent("OK", pendingActuationSequence);
        pendingActuationSequence = 0;
        sensor3StepReferenceValid = false;
      }
      break;
  }
}


// ============================================================
// 13. Serial command parser
// ============================================================

bool verifyAndStripCrc(char* line) {
  char* lastSeparator = strrchr(line, '|');

  if (lastSeparator == nullptr) {
    return false;
  }

  *lastSeparator = '\0';
  const char* receivedCrcText = lastSeparator + 1;

  if (strlen(receivedCrcText) != 4) {
    return false;
  }

  uint16_t receivedCrc = (uint16_t)strtoul(receivedCrcText, nullptr, 16);
  uint16_t calculatedCrc = crc16Ccitt(line);

  return receivedCrc == calculatedCrc;
}


void handleCommand(char* line) {
  if (!verifyAndStripCrc(line)) {
    return;
  }

  char* savePtr = nullptr;

  char* type = strtok_r(line, "|", &savePtr);
  char* sequenceText = strtok_r(nullptr, "|", &savePtr);
  char* operation = strtok_r(nullptr, "|", &savePtr);

  if (
    type == nullptr ||
    sequenceText == nullptr ||
    operation == nullptr ||
    strcmp(type, "C") != 0
  ) {
    return;
  }

  long sequence = atol(sequenceText);

  // ----------------------------------------------------------
  // HELLO
  // C|seq|HELLO|2
  // ----------------------------------------------------------
  if (strcmp(operation, "HELLO") == 0) {
    char* versionText = strtok_r(nullptr, "|", &savePtr);

    if (
      versionText != nullptr &&
      atoi(versionText) == PROTOCOL_VERSION
    ) {
      // Mark the next RUN as a new ROS session start.
      rosStartupPending = true;
      acknowledge(sequence, "OK");
    } else {
      acknowledge(sequence, "ERR_VERSION");
    }
    return;
  }

  // ----------------------------------------------------------
  // SET_OFFSET
  // C|seq|SET_OFFSET|1|15100
  // C|seq|SET_OFFSET|2|7700
  // ----------------------------------------------------------
  if (strcmp(operation, "SET_OFFSET") == 0) {
    char* conveyorText = strtok_r(nullptr, "|", &savePtr);
    char* stepsText = strtok_r(nullptr, "|", &savePtr);

    if (conveyorText == nullptr || stepsText == nullptr) {
      acknowledge(sequence, "ERR");
      return;
    }

    int conveyorId = atoi(conveyorText);
    long steps = atol(stepsText);

    if ((conveyorId == 1 || conveyorId == 2) && steps > 0) {
      conveyors[conveyorId - 1].cameraOffsetSteps = steps;
      acknowledge(sequence, "OK");
    } else {
      acknowledge(sequence, "ERR");
    }
    return;
  }

  // RUN
  // C|seq|RUN|1
  // C|seq|RUN|2
  // ----------------------------------------------------------
  if (strcmp(operation, "RUN") == 0) {
    char* conveyorText = strtok_r(nullptr, "|", &savePtr);

    if (conveyorText == nullptr) {
      acknowledge(sequence, "ERR");
      return;
    }

    int conveyorId = atoi(conveyorText);

    if (conveyorId == 1 || conveyorId == 2) {
      if (rosStartupPending) {
        // ROS start often sends only RUN|1.  Start both conveyors here so
        // Conveyor 2 cannot remain stopped waiting for a separate RUN|2.
        startRosSession();
      } else {
        startContinuousRun(conveyorId - 1);
      }
      acknowledge(sequence, "OK");
    } else {
      acknowledge(sequence, "ERR");
    }
    return;
  }

  // ----------------------------------------------------------
  // STOP
  // C|seq|STOP|1
  // C|seq|STOP|2
  // ----------------------------------------------------------
  if (strcmp(operation, "STOP") == 0) {
    char* conveyorText = strtok_r(nullptr, "|", &savePtr);

    if (conveyorText == nullptr) {
      acknowledge(sequence, "ERR");
      return;
    }

    int conveyorId = atoi(conveyorText);

    if (conveyorId == 1 || conveyorId == 2) {
      stopConveyor(conveyorId - 1);
      acknowledge(sequence, "OK");
    } else {
      acknowledge(sequence, "ERR");
    }
    return;
  }

  // ----------------------------------------------------------
  // ACTUATE
  // 1 = NG / reject
  // 2 = PASS / no physical motion
  // ----------------------------------------------------------
  if (strcmp(operation, "ACTUATE") == 0) {
    char* commandText = strtok_r(nullptr, "|", &savePtr);

    if (commandText == nullptr) {
      acknowledge(sequence, "ERR");
      return;
    }

    int actuatorCommand = atoi(commandText);

    if (actuatorCommand == 1) {
      if (startRejectCycle(sequence)) {
        acknowledge(sequence, "OK");
      } else {
        acknowledge(sequence, "BUSY");
      }
    }
    else if (actuatorCommand == 2) {
      // Normal product: intentionally no servo motion.
      sensor3StepReferenceValid = false;
      acknowledge(sequence, "OK");
      sendActuationEvent("OK", sequence);
    }
    else {
      acknowledge(sequence, "ERR");
    }
    return;
  }

  // ----------------------------------------------------------
  // Optional manual POSITION command for maintenance compatibility.
  // Normal operation starts positioning locally from Sensor 1/2.
  //
  // C|seq|POSITION|conveyor_id|steps
  // ----------------------------------------------------------
  if (strcmp(operation, "POSITION") == 0) {
    char* conveyorText = strtok_r(nullptr, "|", &savePtr);
    char* stepsText = strtok_r(nullptr, "|", &savePtr);

    if (conveyorText == nullptr || stepsText == nullptr) {
      acknowledge(sequence, "ERR");
      return;
    }

    int conveyorId = atoi(conveyorText);
    long steps = atol(stepsText);

    if (
      (conveyorId == 1 || conveyorId == 2) &&
      steps > 0
    ) {
      ConveyorController& conveyor = conveyors[conveyorId - 1];

      if (conveyor.state == CONV_RUNNING) {
        if (startAutomaticPosition(conveyorId - 1, steps, sequence)) {
          acknowledge(sequence, "OK");
        } else {
          acknowledge(sequence, "BUSY");
        }
      } else {
        acknowledge(sequence, "BUSY");
      }
    } else {
      acknowledge(sequence, "ERR");
    }
    return;
  }

  acknowledge(sequence, "ERR_COMMAND");
}


void updateSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      if (rxLength > 0) {
        rxBuffer[rxLength] = '\0';
        handleCommand(rxBuffer);
        rxLength = 0;
      }
      continue;
    }

    if (rxLength < sizeof(rxBuffer) - 1) {
      rxBuffer[rxLength++] = c;
    } else {
      // Overflow: discard the incomplete frame.
      rxLength = 0;
    }
  }
}


// ============================================================
// 14. Arduino setup / main loop
// ============================================================

void setup() {
  Serial.begin(SERIAL_BAUD);

  // TB6600 enable pins
  pinMode(CONV1_EN, OUTPUT);
  pinMode(CONV2_EN, OUTPUT);

  digitalWrite(CONV1_EN, LOW);
  digitalWrite(CONV2_EN, LOW);

  // Conveyor parameters copied from the working test code.
  conveyor1.setMaxSpeed(CONV_MAX_SPEED);
  conveyor1.setAcceleration(CONV_ACCELERATION);
  conveyor1.setSpeed(CONV1_SPEED);

  conveyor2.setMaxSpeed(CONV_MAX_SPEED);
  conveyor2.setAcceleration(CONV_ACCELERATION);
  conveyor2.setSpeed(CONV2_SPEED);

  // HC-SR04
  for (uint8_t i = 0; i < 3; ++i) {
    pinMode(sensors[i].trigPin, OUTPUT);
    pinMode(sensors[i].echoPin, INPUT);
    digitalWrite(sensors[i].trigPin, LOW);
  }

  // Angle-controlled MG996R behavior copied from the test code.
  sorterServo.attach(SERVO_PIN);
  sorterServo.write(SERVO_HOME_ANGLE);

  // Stay stopped until ControlNode completes initialization and sends RUN.
  conveyors[0].state = CONV_STOPPED;
  conveyors[1].state = CONV_STOPPED;

  // No plain-text Serial debug output here.
  // All PC-facing messages use CRC-framed protocol packets.
}


void loop() {
  // These functions are intentionally non-blocking so Conveyor 1,
  // Conveyor 2, ultrasonic sensing, serial communication and servo
  // rejection can progress independently.
  updateSerial();

  updateConveyor(0);
  updateConveyor(1);

  updateUltrasonicSensors();

  updateServo();

  updateEquipmentStateReport();
}

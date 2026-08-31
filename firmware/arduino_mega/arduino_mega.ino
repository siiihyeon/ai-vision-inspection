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
// 4) HC-SR04 #3 detects product arrival and stores Conveyor 2's step.
//    -> Conveyor 2 does NOT stop.
//    -> The servo does NOT move until Master sends ACTUATE|1.
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

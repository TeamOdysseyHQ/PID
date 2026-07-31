#include <Arduino.h>
#include <Encoder.h>
#include <QuickPID.h>
#include <sTune.h>
#include <EEPROM.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// micro-ROS includes
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <rmw_microros/rmw_microros.h>

// ============================================================
// GPS CONFIGURATION (Quectel L86-M33)
// GPS_RX_PIN : Teensy pin that receives data FROM the GPS (connect to L86 TX)
// GPS_TX_PIN : Teensy pin that sends data TO the GPS   (connect to L86 RX)
// ============================================================
#define GPS_RX_PIN    0    // change to your RX pin
#define GPS_TX_PIN    1    // change to your TX pin
#define GPS_BAUD_RATE 9600

// Motor Index Mapping (paired front-to-back):
//  Index 0 = Front Left   | Index 1 = Front Right
//  Index 2 = Middle Left  | Index 3 = Middle Right
//  Index 4 = Back Left    | Index 5 = Back Right
//
// LEFT  side motors: 0, 2, 4
// RIGHT side motors: 1, 3, 5
//
// Hardware wiring reference:
//  Idx  Wheel         PWM Pin  Enc A/B    DIR Pin
//   0   Front Left       4     35 / 34      27
//   1   Front Right      7     41 / 40      26
//   2   Middle Left      3     37 / 36      27
//   3   Middle Right     6     17 / 16      26
//   4   Back Left        2     21 / 20      26
//   5   Back Right       5     39 / 38      27

// ============================================================
// PIN DEFINITIONS
// ============================================================
const int NMOTORS = 6;

const int ENCO_CH_1[] = {35, 41, 37, 17, 21, 39};  // Encoder Channel A
const int ENCO_CH_2[] = {34, 40, 36, 16, 20, 38};  // Encoder Channel B
const int PWM_PIN[] = {4, 7, 3, 6, 2, 5};           // PWM pins
const int DIR_PIN[] = {27, 26, 27, 26, 27, 26};     // Direction pins
const int DIR_POLARITY[] = {-1, -1, -1, -1, -1, -1};   // Invert mirrored motors

const char* MOTOR_LABELS[] = {"FL", "FR", "ML", "MR", "BL", "BR"};
const int LEFT_MOTORS[] = {0, 2, 4};
const int RIGHT_MOTORS[] = {1, 3, 5};

// ============================================================
// ENCODER CONSTANTS
// ============================================================
const int PULSES_PER_REV = 7;
const int GEAR_RATIO[] = {181, 181, 181, 181, 181, 181};
const int COUNTS_PER_REV[] = {
  PULSES_PER_REV * GEAR_RATIO[0] * 4,
  PULSES_PER_REV * GEAR_RATIO[1] * 4,
  PULSES_PER_REV * GEAR_RATIO[2] * 4,
  PULSES_PER_REV * GEAR_RATIO[3] * 4,
  PULSES_PER_REV * GEAR_RATIO[4] * 4,
  PULSES_PER_REV * GEAR_RATIO[5] * 4
};

// Encoder objects
Encoder encoder0(ENCO_CH_1[0], ENCO_CH_2[0]);
Encoder encoder1(ENCO_CH_1[1], ENCO_CH_2[1]);
Encoder encoder2(ENCO_CH_1[2], ENCO_CH_2[2]);
Encoder encoder3(ENCO_CH_1[3], ENCO_CH_2[3]);
Encoder encoder4(ENCO_CH_1[4], ENCO_CH_2[4]);
Encoder encoder5(ENCO_CH_1[5], ENCO_CH_2[5]);

// ============================================================
// EEPROM CONFIG (per-motor PID gains)
// ============================================================
#define EEPROM_ADDR 0
#define EEPROM_MAGIC 0xA6D00001

struct PIDConfig {
  uint32_t magic;
  float Kp[6];
  float Ki[6];
  float Kd[6];
};

// Default PID gains
const float DEFAULT_KP = 2.0f;
const float DEFAULT_KI = 5.0f;
const float DEFAULT_KD = 0.1f;

// Per-motor PID gain storage
float pidKp[NMOTORS];
float pidKi[NMOTORS];
float pidKd[NMOTORS];

// ============================================================
// RPM CALCULATION VARIABLES
// ============================================================
unsigned long prevTime[NMOTORS] = {0};
long prevEncoderPos[NMOTORS] = {0};
float motorRPM[NMOTORS] = {0};
float motorRPM_filtered[NMOTORS] = {0};

// ============================================================
// MOTOR CONTROL VARIABLES
// ============================================================
int motorPWM[NMOTORS] = {0};
int targetPWM[NMOTORS] = {0};
int motorDirection[NMOTORS] = {1, 1, 1, 1, 1, 1};
bool isStopping[NMOTORS] = {false};
bool isRampingDown = false;

const int PWM_RAMP_STEP = 5;
const int PWM_RAMP_DELAY = 20;
unsigned long lastRampTime[NMOTORS] = {0};

const int RAMPDOWN_STEP = 5;
const unsigned long RAMPDOWN_INTERVAL = 500;
unsigned long lastRampDownTime = 0;

// ============================================================
// PWM SLEW RATE LIMITER
// Adjustable: raise STEP or lower INTERVAL = faster ramp; reverse = softer
// ============================================================
const int          PWM_SLEW_STEP        = 8;    // max PWM counts changed per interval
const unsigned long PWM_SLEW_INTERVAL_MS = 12;  // ms between each step per motor

int           currentAppliedPWM[NMOTORS]  = {0};              // PWM actually written to hardware
int           targetDirection[NMOTORS]    = {1, 1, 1, 1, 1, 1}; // desired direction (set by PID)
bool          pendingDirChange[NMOTORS]   = {false};            // waiting for PWM→0 before DIR flip
unsigned long lastSlewTime[NMOTORS]       = {0};

// ============================================================
// MICRO-ROS VARIABLES
// ============================================================
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_subscription_t subscriber;
rcl_publisher_t publisher;
rcl_publisher_t gps_publisher;
rclc_executor_t executor;

// Message objects
std_msgs__msg__Float32MultiArray sub_msg;
std_msgs__msg__Float32MultiArray pub_msg;
sensor_msgs__msg__NavSatFix gps_msg;
static char gps_frame_id[] = "gps";

// Backing arrays for Float32MultiArray data
float sub_data[NMOTORS];
float pub_data[NMOTORS];

// Layout dimension backing (1D array of 6)
std_msgs__msg__MultiArrayDimension sub_dim[1];
std_msgs__msg__MultiArrayDimension pub_dim[1];

// micro-ROS connection state
bool micro_ros_connected = false;
unsigned long last_agent_ping = 0;
const unsigned long AGENT_PING_INTERVAL_MS = 2000;

// Subscriber watchdog — stop motors if no message received
unsigned long lastSubMsgTime = 0;
const unsigned long SUB_WATCHDOG_TIMEOUT_MS = 3000;
bool watchdogTriggered = false;

// ============================================================
// GPS VARIABLES
// ============================================================
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPSPlus gps;
unsigned long lastGpsPublishTime = 0;
const unsigned long GPS_PUBLISH_INTERVAL_MS = 200;  // publish at up to 5 Hz

// Error check macro — on failure, enter error loop with LED blink
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if (temp_rc != RCL_RET_OK) { error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; (void)temp_rc; }

// ============================================================
// FORWARD DECLARATIONS
// ============================================================
void stopMotor(int motorIndex);
void disablePID(int motorIndex);
void enablePIDForMotor(int motorIndex, float targetRPM);
void cancelAutoTune();
void setMotor(int motorIndex, int pwmValue, int direction);

// ============================================================
// PID CONTROL VARIABLES
// ============================================================
float pidInput[NMOTORS] = {0};
float pidOutput[NMOTORS] = {0};
float pidSetpoint[NMOTORS] = {0};
int pidDirection[NMOTORS] = {1, 1, 1, 1, 1, 1};  // direction from RPM sign
bool pidEnabled[NMOTORS] = {false};

const uint32_t PID_SAMPLE_TIME_US = 50000;  // 50ms

// 6 QuickPID objects — using simplified constructor, then SetTunings in setup
QuickPID pid0(&pidInput[0], &pidOutput[0], &pidSetpoint[0]);
QuickPID pid1(&pidInput[1], &pidOutput[1], &pidSetpoint[1]);
QuickPID pid2(&pidInput[2], &pidOutput[2], &pidSetpoint[2]);
QuickPID pid3(&pidInput[3], &pidOutput[3], &pidSetpoint[3]);
QuickPID pid4(&pidInput[4], &pidOutput[4], &pidSetpoint[4]);
QuickPID pid5(&pidInput[5], &pidOutput[5], &pidSetpoint[5]);

QuickPID* pidControllers[] = {&pid0, &pid1, &pid2, &pid3, &pid4, &pid5};

// ============================================================
// AUTOTUNE (sTune) VARIABLES
// ============================================================
float tuneInput = 0;
float tuneOutput = 0;

sTune tuner = sTune(&tuneInput, &tuneOutput, sTune::Mixed_PID, sTune::directIP, sTune::printALL);

int autoTuneMotor = -1;        // Which motor is being autotuned (-1 = none)
bool autoTuneRunning = false;
unsigned long autoTuneStartTime = 0;
const unsigned long AUTOTUNE_TIMEOUT_MS = 20000;  // 20s safety timeout

// sTune configuration for DC motors
const float AT_INPUT_SPAN = 60.0f;    // Max RPM range
const float AT_OUTPUT_SPAN = 255.0f;  // PWM range
const float AT_OUTPUT_START = 0.0f;   // Start PWM
const float AT_OUTPUT_STEP = 150.0f;  // Step PWM during test
const uint32_t AT_TEST_TIME_SEC = 5;  // Test duration
const uint32_t AT_SETTLE_TIME_SEC = 2; // Settle time
const uint16_t AT_SAMPLES = 300;       // Sample count

// ============================================================
// FUNCTION: micro-ROS error loop (blink LED rapidly)
// ============================================================
void error_loop() {
  while (1) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

// ============================================================
// FUNCTION: micro-ROS subscription callback
// ============================================================
void motorRpmCallback(const void *msgin) {
  const std_msgs__msg__Float32MultiArray *msg =
      (const std_msgs__msg__Float32MultiArray *)msgin;

  if (msg == NULL || msg->data.size != NMOTORS) return;

  lastSubMsgTime = millis();
  watchdogTriggered = false;

  for (int i = 0; i < NMOTORS; i++) {
    float targetRPM = msg->data.data[i];

    if (fabs(targetRPM) < 0.5f) {
      // Target ~0: stop this motor
      if (pidEnabled[i]) {
        disablePID(i);
        motorPWM[i] = 0;
      }
    } else {
      enablePIDForMotor(i, targetRPM);
    }
  }
}

// ============================================================
// FUNCTION: Initialize micro-ROS messages (Float32MultiArray)
// ============================================================
void initMessages() {
  // Subscriber message
  sub_msg.data.data = sub_data;
  sub_msg.data.size = NMOTORS;
  sub_msg.data.capacity = NMOTORS;
  sub_msg.layout.dim.data = sub_dim;
  sub_msg.layout.dim.size = 1;
  sub_msg.layout.dim.capacity = 1;
  sub_msg.layout.dim.data[0].size = NMOTORS;
  sub_msg.layout.dim.data[0].stride = NMOTORS;
  sub_msg.layout.data_offset = 0;

  // Publisher message
  pub_msg.data.data = pub_data;
  pub_msg.data.size = NMOTORS;
  pub_msg.data.capacity = NMOTORS;
  pub_msg.layout.dim.data = pub_dim;
  pub_msg.layout.dim.size = 1;
  pub_msg.layout.dim.capacity = 1;
  pub_msg.layout.dim.data[0].size = NMOTORS;
  pub_msg.layout.dim.data[0].stride = NMOTORS;
  pub_msg.layout.data_offset = 0;
}

// ============================================================
// FUNCTION: Setup micro-ROS node, publisher, subscriber, executor
// ============================================================
void setupMicroRos() {
  // USB serial transport
  set_microros_transports();

  // Wait for agent to be available (blink LED while waiting)
  while (rmw_uros_ping_agent(200, 3) != RMW_RET_OK) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(300);
  }
  digitalWrite(LED_BUILTIN, HIGH);

  allocator = rcl_get_default_allocator();

  // Create init_options and support
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "teensy_motor_node", "", &support));

  // Create subscriber on "teensy_motor_rpm" with best-effort QoS
  RCCHECK(rclc_subscription_init_best_effort(
      &subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "teensy_motor_rpm"));

  // Create publisher on "motor_rpms" with best-effort QoS
  RCCHECK(rclc_publisher_init_best_effort(
      &publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "motor_rpms"));

  // Create GPS publisher on "/gps/fix" with best-effort QoS
  RCCHECK(rclc_publisher_init_best_effort(
      &gps_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix),
      "/gps/fix"));

  // Initialize GPS message header frame_id
  gps_msg.header.frame_id.data     = gps_frame_id;
  gps_msg.header.frame_id.size     = sizeof(gps_frame_id) - 1;
  gps_msg.header.frame_id.capacity = sizeof(gps_frame_id);

  // Initialize messages
  initMessages();

  // Create executor with 1 subscription handle
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &subscriber, &sub_msg, &motorRpmCallback, ON_NEW_DATA));

  micro_ros_connected = true;
  lastSubMsgTime = millis();
}

// ============================================================
// FUNCTION: Publish current motor RPMs
// ============================================================
void publishMotorRpms() {
  if (!micro_ros_connected) return;

  for (int i = 0; i < NMOTORS; i++) {
    // Publish signed RPM (apply direction sign)
    float signedRPM = motorRPM_filtered[i];
    pub_msg.data.data[i] = signedRPM;
  }

  RCSOFTCHECK(rcl_publish(&publisher, &pub_msg, NULL));
}

// ============================================================
// FUNCTION: Stop all motors (safety)
// ============================================================
void emergencyStopAll() {
  for (int i = 0; i < NMOTORS; i++) {
    disablePID(i);
    motorPWM[i] = 0;
    currentAppliedPWM[i] = 0;  // bypass slew — instant cut
    pendingDirChange[i] = false;
    stopMotor(i);
    isStopping[i] = false;
  }
  isRampingDown = false;
  cancelAutoTune();
}

// ============================================================
// FUNCTION: Check subscriber watchdog — stop if no messages
// ============================================================
void checkSubscriberWatchdog() {
  if (!micro_ros_connected) return;

  // Only trigger if at least one motor has PID enabled
  bool anyEnabled = false;
  for (int i = 0; i < NMOTORS; i++) {
    if (pidEnabled[i]) { anyEnabled = true; break; }
  }

  if (anyEnabled && (millis() - lastSubMsgTime > SUB_WATCHDOG_TIMEOUT_MS)) {
    if (!watchdogTriggered) {
      emergencyStopAll();
      watchdogTriggered = true;
    }
  }
}

// ============================================================
// FUNCTION: Get encoder position
// ============================================================
long getEncoderPos(int motorIndex) {
  switch (motorIndex) {
    case 0: return encoder0.read();
    case 1: return encoder1.read();
    case 2: return encoder2.read();
    case 3: return encoder3.read();
    case 4: return encoder4.read();
    case 5: return encoder5.read();
    default: return 0;
  }
}

// ============================================================
// FUNCTION: Set motor speed and direction
// ============================================================
void setMotor(int motorIndex, int pwmValue, int direction) {
  pwmValue = constrain(pwmValue, 0, 255);
  int effectiveDir = direction * DIR_POLARITY[motorIndex];
  if (effectiveDir > 0) {
    digitalWrite(DIR_PIN[motorIndex], HIGH);
  } else {
    digitalWrite(DIR_PIN[motorIndex], LOW);
  }
  analogWrite(PWM_PIN[motorIndex], pwmValue);
}

// ============================================================
// FUNCTION: Stop motor
// ============================================================
void stopMotor(int motorIndex) {
  analogWrite(PWM_PIN[motorIndex], 0);
}

// ============================================================
// FUNCTION: Disable PID for a motor (revert to manual)
// ============================================================
void disablePID(int motorIndex) {
  pidEnabled[motorIndex] = false;
  pidSetpoint[motorIndex] = 0;
  pidOutput[motorIndex] = 0;
  pidControllers[motorIndex]->SetMode(QuickPID::Control::manual);
}

// ============================================================
// FUNCTION: Enable PID for a motor with target RPM
// ============================================================
void enablePIDForMotor(int motorIndex, float targetRPM) {
  if (targetRPM >= 0) {
    pidDirection[motorIndex] = 1;
    pidSetpoint[motorIndex] = targetRPM;
  } else {
    pidDirection[motorIndex] = -1;
    pidSetpoint[motorIndex] = -targetRPM;  // absolute value
  }

  if (!pidEnabled[motorIndex]) {
    // Fresh enable: reset PID state completely for clean start
    pidOutput[motorIndex] = 0;
    motorPWM[motorIndex] = 0;
    pidControllers[motorIndex]->SetMode(QuickPID::Control::manual);
    pidControllers[motorIndex]->Reset();  // clears outputSum, pTerm, iTerm, dTerm
    pidControllers[motorIndex]->SetMode(QuickPID::Control::automatic); // calls Initialize()
  }
  // else: already running — just update setpoint for bumpless transfer

  pidEnabled[motorIndex] = true;
  isStopping[motorIndex] = false;
}

// ============================================================
// EEPROM: Load PID config
// ============================================================
void loadPIDConfig() {
  PIDConfig config;
  EEPROM.get(EEPROM_ADDR, config);

  if (config.magic == EEPROM_MAGIC) {
    for (int i = 0; i < NMOTORS; i++) {
      pidKp[i] = config.Kp[i];
      pidKi[i] = config.Ki[i];
      pidKd[i] = config.Kd[i];
      pidControllers[i]->SetTunings(pidKp[i], pidKi[i], pidKd[i]);
    }
    // EEPROM: Loaded per-motor PID gains
  } else {
    for (int i = 0; i < NMOTORS; i++) {
      pidKp[i] = DEFAULT_KP;
      pidKi[i] = DEFAULT_KI;
      pidKd[i] = DEFAULT_KD;
      pidControllers[i]->SetTunings(pidKp[i], pidKi[i], pidKd[i]);
    }
    // EEPROM: No valid config, using defaults
  }
}

// ============================================================
// EEPROM: Save PID config
// ============================================================
void savePIDConfig() {
  PIDConfig config;
  config.magic = EEPROM_MAGIC;
  for (int i = 0; i < NMOTORS; i++) {
    config.Kp[i] = pidKp[i];
    config.Ki[i] = pidKi[i];
    config.Kd[i] = pidKd[i];
  }
  EEPROM.put(EEPROM_ADDR, config);
}

// ============================================================
// FUNCTION: Gentle stop
// ============================================================
void gentleStopMotor(int motorIndex) {
  disablePID(motorIndex);
  isStopping[motorIndex] = true;
  targetPWM[motorIndex] = 0;
}

void updatePWMRamp() {
  unsigned long currentTime = millis();
  for (int i = 0; i < NMOTORS; i++) {
    if (isStopping[i] && motorPWM[i] > 0) {
      if (currentTime - lastRampTime[i] >= PWM_RAMP_DELAY) {
        motorPWM[i] -= PWM_RAMP_STEP;
        if (motorPWM[i] <= 0) {
          motorPWM[i] = 0;
          isStopping[i] = false;
        }
        lastRampTime[i] = currentTime;
      }
    }
  }
}

void gentleStopAllMotors() {
  for (int i = 0; i < NMOTORS; i++) {
    gentleStopMotor(i);
  }
}

// ============================================================
// FUNCTION: Ramp-down
// ============================================================
void startRampDown() {
  for (int i = 0; i < NMOTORS; i++) disablePID(i);
  isRampingDown = true;
  lastRampDownTime = millis();
}

void updateRampDown() {
  if (!isRampingDown) return;
  unsigned long currentTime = millis();
  if (currentTime - lastRampDownTime >= RAMPDOWN_INTERVAL) {
    bool allZero = true;
    for (int i = 0; i < NMOTORS; i++) {
      if (motorPWM[i] > 0) {
        motorPWM[i] -= RAMPDOWN_STEP;
        if (motorPWM[i] < 0) motorPWM[i] = 0;
      }
      if (motorPWM[i] > 0) allZero = false;
    }
    lastRampDownTime = currentTime;
    if (allZero) {
      isRampingDown = false;
    }
  }
}

// ============================================================
// FUNCTION: PWM Slew Rate Limiter
// Intercepts every hardware write — steps currentAppliedPWM[] toward
// motorPWM[] at a capped rate. Handles direction reversals safely by
// coasting to 0 before flipping the DIR pin, then ramping back up.
// Emergency stop bypasses this by zeroing currentAppliedPWM[] directly.
// Tune: PWM_SLEW_STEP / PWM_SLEW_INTERVAL_MS at the top of the file.
// ============================================================
void updatePWMSlew() {
  unsigned long now = millis();
  for (int i = 0; i < NMOTORS; i++) {
    if (autoTuneRunning && i == autoTuneMotor) continue;  // autotune manages its own output

    if (now - lastSlewTime[i] < PWM_SLEW_INTERVAL_MS) continue;
    lastSlewTime[i] = now;

    // Resolve the desired PWM for this tick.
    // If a direction change is pending, force target to 0 until we've coasted down.
    int desiredPWM = motorPWM[i];
    if (pendingDirChange[i]) {
      desiredPWM = 0;
    }

    // Step currentAppliedPWM toward desiredPWM
    if (currentAppliedPWM[i] < desiredPWM) {
      currentAppliedPWM[i] += PWM_SLEW_STEP;
      if (currentAppliedPWM[i] > desiredPWM) currentAppliedPWM[i] = desiredPWM;
    } else if (currentAppliedPWM[i] > desiredPWM) {
      currentAppliedPWM[i] -= PWM_SLEW_STEP;
      if (currentAppliedPWM[i] < desiredPWM) currentAppliedPWM[i] = desiredPWM;
    }

    // Once coasted to 0, flip the direction and allow ramp-up next tick
    if (pendingDirChange[i] && currentAppliedPWM[i] == 0) {
      motorDirection[i] = targetDirection[i];
      pendingDirChange[i] = false;
    }
  }
}

// ============================================================
// FUNCTION: Calculate RPM with low-pass filter
// ============================================================
void calculateRPM() {
  for (int i = 0; i < NMOTORS; i++) {
    unsigned long currentTime = micros();
    float deltaTime = ((float)(currentTime - prevTime[i])) / 1.0e6;

    if (deltaTime >= 0.05) {  // 50ms
      long currentPos = getEncoderPos(i);
      long deltaPos = currentPos - prevEncoderPos[i];

      float rawRPM = (deltaPos / deltaTime) / COUNTS_PER_REV[i] * 60.0;

      motorRPM_filtered[i] = 0.3 * rawRPM + 0.7 * motorRPM_filtered[i];
      motorRPM[i] = motorRPM_filtered[i];

      // Feed absolute RPM into PID input
      pidInput[i] = fabs(motorRPM_filtered[i]);

      // Feed autotune input if this motor is being autotuned
      if (autoTuneRunning && i == autoTuneMotor) {
        tuneInput = pidInput[i];
      }

      prevEncoderPos[i] = currentPos;
      prevTime[i] = currentTime;
    }
  }
}

// ============================================================
// FUNCTION: Cancel autotune
// ============================================================
void cancelAutoTune() {
  if (autoTuneRunning && autoTuneMotor >= 0) {
    motorPWM[autoTuneMotor] = 0;
    stopMotor(autoTuneMotor);
    disablePID(autoTuneMotor);
  }
  autoTuneRunning = false;
  autoTuneMotor = -1;
}

// ============================================================
// FUNCTION: Start autotune on a specific motor
// ============================================================
void startAutoTune(int motorIdx) {
  if (autoTuneRunning) return;
  if (motorIdx < 0 || motorIdx >= NMOTORS) return;

  // Disable PID on all motors, stop all
  for (int i = 0; i < NMOTORS; i++) {
    disablePID(i);
    motorPWM[i] = 0;
    stopMotor(i);
  }

  autoTuneMotor = motorIdx;
  autoTuneRunning = true;
  autoTuneStartTime = millis();
  tuneInput = 0;
  tuneOutput = 0;

  // Configure sTune
  tuner.Configure(AT_INPUT_SPAN, AT_OUTPUT_SPAN, AT_OUTPUT_START, AT_OUTPUT_STEP,
                  AT_TEST_TIME_SEC, AT_SETTLE_TIME_SEC, AT_SAMPLES);
  tuner.SetEmergencyStop(AT_INPUT_SPAN);  // e-stop if RPM exceeds span
}

// ============================================================
// FUNCTION: Update autotune (called every loop)
// ============================================================
void updateAutoTune() {
  if (!autoTuneRunning || autoTuneMotor < 0) return;

  // Timeout safety
  if (millis() - autoTuneStartTime > AUTOTUNE_TIMEOUT_MS) {
    cancelAutoTune();
    return;
  }

  // Run sTune — softPwm manages its own timing
  switch (tuner.Run()) {
    case tuner.sample: // ready for new sample
      tuneInput = pidInput[autoTuneMotor];
      break;

    case tuner.tunings: { // tuning complete
      float kp, ki, kd;
      tuner.GetAutoTunings(&kp, &ki, &kd);

      // Apply to per-motor gains
      pidKp[autoTuneMotor] = kp;
      pidKi[autoTuneMotor] = ki;
      pidKd[autoTuneMotor] = kd;
      pidControllers[autoTuneMotor]->SetTunings(kp, ki, kd);

      // Stop motor after test
      motorPWM[autoTuneMotor] = 0;
      stopMotor(autoTuneMotor);

      autoTuneRunning = false;
      autoTuneMotor = -1;
      break;
    }

    default: // test running — apply tuneOutput to motor
      break;
  }

  // While test is running, apply tuneOutput to the motor
  if (autoTuneRunning && autoTuneMotor >= 0) {
    motorPWM[autoTuneMotor] = constrain((int)tuneOutput, 0, 255);
    motorDirection[autoTuneMotor] = 1;  // forward during autotune
    setMotor(autoTuneMotor, motorPWM[autoTuneMotor], motorDirection[autoTuneMotor]);
  }
}

// ============================================================
// FUNCTION: Feed GPS serial bytes into TinyGPS++ parser
// ============================================================
void readGPS() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
}

// ============================================================
// FUNCTION: Publish GPS NavSatFix to /gps/fix
// ============================================================
void publishGpsFix() {
  if (!micro_ros_connected) return;

  gps_msg.header.stamp.sec     = (int32_t)(millis() / 1000);
  gps_msg.header.stamp.nanosec = (uint32_t)((millis() % 1000) * 1000000UL);

  if (gps.location.isValid()) {
    gps_msg.latitude  = gps.location.lat();
    gps_msg.longitude = gps.location.lng();
    gps_msg.altitude  = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
    gps_msg.status.status  = 0;   // STATUS_FIX
    gps_msg.status.service = 1;   // SERVICE_GPS
  } else {
    gps_msg.latitude  = 0.0;
    gps_msg.longitude = 0.0;
    gps_msg.altitude  = 0.0;
    gps_msg.status.status  = -1;  // STATUS_NO_FIX
    gps_msg.status.service = 1;   // SERVICE_GPS
  }

  gps_msg.position_covariance_type = 0;  // COVARIANCE_TYPE_UNKNOWN
  for (int i = 0; i < 9; i++) gps_msg.position_covariance[i] = 0.0;

  RCSOFTCHECK(rcl_publish(&gps_publisher, &gps_msg, NULL));
}

// ============================================================
// FUNCTION: PID compute step (called every loop)
// ============================================================
void updatePIDControl() {
  for (int i = 0; i < NMOTORS; i++) {
    if (!pidEnabled[i]) continue;
    if (autoTuneRunning && i == autoTuneMotor) continue;  // skip during autotune

    if (pidControllers[i]->Compute()) {
      motorPWM[i] = constrain((int)pidOutput[i], 0, 255);
      // Feed desired direction to slew layer; motorDirection[] is managed by updatePWMSlew()
      if (targetDirection[i] != pidDirection[i]) {
        targetDirection[i] = pidDirection[i];
        pendingDirChange[i] = true;
      }
    }
  }
}



// ============================================================
// SETUP
// ============================================================
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Initialize motor pins
  for (int i = 0; i < NMOTORS; i++) {
    pinMode(ENCO_CH_1[i], INPUT);
    pinMode(ENCO_CH_2[i], INPUT);
    pinMode(PWM_PIN[i], OUTPUT);
    pinMode(DIR_PIN[i], OUTPUT);
    stopMotor(i);
  }

  // PWM frequency 30kHz
  for (int i = 0; i < NMOTORS; i++) {
    analogWriteFrequency(PWM_PIN[i], 1000);
  }

  // Initialize PID controllers
  for (int i = 0; i < NMOTORS; i++) {
    pidControllers[i]->SetOutputLimits(0, 255);
    pidControllers[i]->SetSampleTimeUs(PID_SAMPLE_TIME_US);
    pidControllers[i]->SetMode(QuickPID::Control::manual);
  }

  // Load PID gains from EEPROM (or use defaults)
  loadPIDConfig();

  // Initialize RPM timing
  for (int i = 0; i < NMOTORS; i++) {
    prevTime[i] = micros();
  }

  // Initialize GPS serial (Quectel L86-M33)
  gpsSerial.begin(GPS_BAUD_RATE);

  // Initialize micro-ROS (blocks until agent is available)
  setupMicroRos();
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  // 1. Calculate RPM from encoders
  calculateRPM();

  // 2. Update autotune (if running)
  updateAutoTune();

  // 3. Update PID control
  updatePIDControl();

  // 4. Update PWM ramping for gentle stops
  updatePWMRamp();

  // 5. Update ramp-down
  updateRampDown();

  // 5.5. Update PWM slew rate limiter (steps currentAppliedPWM toward motorPWM)
  updatePWMSlew();

  // 6. Apply motor settings (skip autotune motor — handled in updateAutoTune)
  for (int i = 0; i < NMOTORS; i++) {
    if (autoTuneRunning && i == autoTuneMotor) continue;
    setMotor(i, currentAppliedPWM[i], motorDirection[i]);
  }

  // 7. Publish current motor RPMs every 50ms (matches PID sample rate)
  static unsigned long lastPublishTime = 0;
  if (millis() - lastPublishTime >= 50) {
    publishMotorRpms();
    lastPublishTime = millis();
  }

  // 8. Read GPS serial data and publish NavSatFix at up to 5 Hz
  readGPS();
  if (millis() - lastGpsPublishTime >= GPS_PUBLISH_INTERVAL_MS) {
    publishGpsFix();
    lastGpsPublishTime = millis();
  }

  // 9. Spin micro-ROS executor (process incoming messages, non-blocking)
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));

  // 10. Check subscriber watchdog — stop motors if no messages received
  checkSubscriberWatchdog();

  // 11. Periodically ping agent — if lost, emergency stop
  if (millis() - last_agent_ping >= AGENT_PING_INTERVAL_MS) {
    if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
      if (micro_ros_connected) {
        emergencyStopAll();
        micro_ros_connected = false;
      }
    } else {
      micro_ros_connected = true;
    }
    last_agent_ping = millis();
  }

  delay(1);  // Small yield
}

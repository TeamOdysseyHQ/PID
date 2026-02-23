#include <Arduino.h>
#include <Encoder.h>
#include <QuickPID.h>
#include <sTune.h>
#include <EEPROM.h>

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
const int DIR_PIN[] = {27, 26, 27, 26, 26, 27};     // Direction pins
const int DIR_POLARITY[] = {-1, 1, -1, 1, 1, -1};   // Invert mirrored motors

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
    Serial.println("EEPROM: Loaded per-motor PID gains");
  } else {
    for (int i = 0; i < NMOTORS; i++) {
      pidKp[i] = DEFAULT_KP;
      pidKi[i] = DEFAULT_KI;
      pidKd[i] = DEFAULT_KD;
      pidControllers[i]->SetTunings(pidKp[i], pidKi[i], pidKd[i]);
    }
    Serial.println("EEPROM: No valid config found, using defaults (Kp=2.0, Ki=5.0, Kd=0.1)");
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
  Serial.println("EEPROM: Saved per-motor PID gains");
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
  Serial.println("RAMPDOWN started: reducing all motors by 5 PWM every 0.5s");
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
      Serial.println("RAMPDOWN complete: all motors at 0");
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
    Serial.print("AUTOTUNE cancelled for M");
    Serial.println(autoTuneMotor);
  }
  autoTuneRunning = false;
  autoTuneMotor = -1;
}

// ============================================================
// FUNCTION: Start autotune on a specific motor
// ============================================================
void startAutoTune(int motorIdx) {
  if (autoTuneRunning) {
    Serial.println("AUTOTUNE already running! Send AUTOTUNECANCEL first.");
    return;
  }
  if (motorIdx < 0 || motorIdx >= NMOTORS) {
    Serial.println("Invalid motor index for AUTOTUNE");
    return;
  }

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

  Serial.print("AUTOTUNE started on M");
  Serial.print(motorIdx);
  Serial.print("(");
  Serial.print(MOTOR_LABELS[motorIdx]);
  Serial.println(") — all other motors stopped. Please wait...");
}

// ============================================================
// FUNCTION: Update autotune (called every loop)
// ============================================================
void updateAutoTune() {
  if (!autoTuneRunning || autoTuneMotor < 0) return;

  // Timeout safety
  if (millis() - autoTuneStartTime > AUTOTUNE_TIMEOUT_MS) {
    Serial.println("AUTOTUNE timeout! Aborting.");
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

      Serial.println("=== AUTOTUNE COMPLETE ===");
      Serial.print("Motor M");
      Serial.print(autoTuneMotor);
      Serial.print("(");
      Serial.print(MOTOR_LABELS[autoTuneMotor]);
      Serial.print(") => Kp=");
      Serial.print(kp, 4);
      Serial.print(" Ki=");
      Serial.print(ki, 4);
      Serial.print(" Kd=");
      Serial.println(kd, 4);
      Serial.println("Use SAVE to persist these gains to EEPROM.");

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
// FUNCTION: PID compute step (called every loop)
// ============================================================
void updatePIDControl() {
  for (int i = 0; i < NMOTORS; i++) {
    if (!pidEnabled[i]) continue;
    if (autoTuneRunning && i == autoTuneMotor) continue;  // skip during autotune

    if (pidControllers[i]->Compute()) {
      motorPWM[i] = constrain((int)pidOutput[i], 0, 255);
      motorDirection[i] = pidDirection[i];
    }
  }
}

// ============================================================
// FUNCTION: Print PID status for all motors
// ============================================================
void printPIDStatus() {
  Serial.println("=== PID STATUS ===");
  for (int i = 0; i < NMOTORS; i++) {
    Serial.print("  M");
    Serial.print(i);
    Serial.print("(");
    Serial.print(MOTOR_LABELS[i]);
    Serial.print(") Kp=");
    Serial.print(pidKp[i], 4);
    Serial.print(" Ki=");
    Serial.print(pidKi[i], 4);
    Serial.print(" Kd=");
    Serial.print(pidKd[i], 4);
    Serial.print(" | ");
    if (pidEnabled[i]) {
      Serial.print("[PID] target=");
      Serial.print(pidSetpoint[i], 1);
      Serial.print(" RPM");
    } else {
      Serial.print("[MANUAL]");
    }
    Serial.println();
  }
}

// ============================================================
// FUNCTION: Parse RPM value from command string at given position
// ============================================================
float parseRPMValue(const String& cmd, int startPos) {
  String val = cmd.substring(startPos);
  val.trim();
  return val.toFloat();
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Initialize pins
  for (int i = 0; i < NMOTORS; i++) {
    pinMode(ENCO_CH_1[i], INPUT);
    pinMode(ENCO_CH_2[i], INPUT);
    pinMode(PWM_PIN[i], OUTPUT);
    pinMode(DIR_PIN[i], OUTPUT);
    stopMotor(i);
  }

  // PWM frequency 30kHz
  for (int i = 0; i < NMOTORS; i++) {
    analogWriteFrequency(PWM_PIN[i], 30000);
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

  Serial.println("===================================");
  Serial.println("  PID Motor Control with AutoTune");
  Serial.println("===================================");
  Serial.println("Motor: 0=FL, 1=FR, 2=ML, 3=MR, 4=BL, 5=BR");
  Serial.println("Left: 0,2,4 | Right: 1,3,5");
  Serial.println();
  Serial.println("--- Manual PWM Commands ---");
  Serial.println("  M<n> P<pwm> D<dir>  Single motor manual control");
  Serial.println("  ALL P<pwm> D<dir>   All motors manual");
  Serial.println("  LEFT P<pwm> D<dir>  Left side manual");
  Serial.println("  RIGHT P<pwm> D<dir> Right side manual");
  Serial.println("  S<n>                Gentle stop motor");
  Serial.println("  X<n>                Immediate stop motor");
  Serial.println("  STOPALL             Gentle stop all");
  Serial.println("  RAMPDOWN            Ramp down all (5 PWM/0.5s)");
  Serial.println("  XALL                Immediate stop all");
  Serial.println();
  Serial.println("--- PID RPM Commands ---");
  Serial.println("  RPM<n> <rpm>        Set motor to target RPM (neg=reverse)");
  Serial.println("  RPMALL <rpm>        Set all motors to target RPM");
  Serial.println("  RPMLEFT <rpm>       Left side target RPM");
  Serial.println("  RPMRIGHT <rpm>      Right side target RPM");
  Serial.println("  PIDOFF              Disable PID all motors");
  Serial.println("  PIDOFF<n>           Disable PID single motor");
  Serial.println();
  Serial.println("--- PID Tuning Commands ---");
  Serial.println("  TUNE<n> <Kp> <Ki> <Kd>   Set gains for motor n");
  Serial.println("  TUNEALL <Kp> <Ki> <Kd>   Set gains for all motors");
  Serial.println("  AUTOTUNE<n>               AutoTune motor n (sTune)");
  Serial.println("  AUTOTUNECANCEL            Cancel running autotune");
  Serial.println("  PIDSTATUS                 Print all PID gains & modes");
  Serial.println();
  Serial.println("--- EEPROM Commands ---");
  Serial.println("  SAVE                Save PID gains to EEPROM");
  Serial.println("  LOAD                Load PID gains from EEPROM");
  Serial.println();
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

  // 6. Apply motor settings (skip autotune motor — handled in updateAutoTune)
  for (int i = 0; i < NMOTORS; i++) {
    if (autoTuneRunning && i == autoTuneMotor) continue;
    setMotor(i, motorPWM[i], motorDirection[i]);
  }

  // 7. Print status every 500ms
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime >= 500) {
    for (int i = 0; i < NMOTORS; i++) {
      Serial.print("M");
      Serial.print(i);
      Serial.print("(");
      Serial.print(MOTOR_LABELS[i]);
      Serial.print(") ");

      if (autoTuneRunning && i == autoTuneMotor) {
        Serial.print("[AUTOTUNE] ");
      } else if (pidEnabled[i]) {
        Serial.print("[PID tgt=");
        Serial.print(pidSetpoint[i], 1);
        Serial.print("] ");
      } else {
        Serial.print("[MAN] ");
      }

      Serial.print("RPM:");
      Serial.print(motorRPM[i], 2);
      if (pidEnabled[i]) {
        Serial.print(" Out:");
        Serial.print(pidOutput[i], 1);
        Serial.print(" I:");
        Serial.print(pidControllers[i]->GetOutputSum(), 1);
      }
      Serial.print(" PWM:");
      Serial.print(motorPWM[i]);
      Serial.println();
    }
    Serial.println();
    lastPrintTime = millis();
  }

  // ============================================================
  // 8. SERIAL COMMAND PARSER
  // ============================================================
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    // ---- AUTOTUNE commands (check first — longest prefix match) ----
    if (command.startsWith("AUTOTUNECANCEL")) {
      cancelAutoTune();

    } else if (command.startsWith("AUTOTUNE")) {
      int motorIdx = command.substring(8).toInt();
      startAutoTune(motorIdx);

    // ---- PIDSTATUS ----
    } else if (command == "PIDSTATUS") {
      printPIDStatus();

    // ---- PIDOFF ----
    } else if (command.startsWith("PIDOFF")) {
      String rest = command.substring(6);
      rest.trim();
      if (rest.length() == 0) {
        // Disable all
        for (int i = 0; i < NMOTORS; i++) {
          disablePID(i);
          motorPWM[i] = 0;
        }
        Serial.println("PID disabled on ALL motors");
      } else {
        int idx = rest.toInt();
        if (idx >= 0 && idx < NMOTORS) {
          disablePID(idx);
          motorPWM[idx] = 0;
          Serial.print("PID disabled on M");
          Serial.println(idx);
        }
      }

    // ---- RPM commands ----
    } else if (command.startsWith("RPMALL")) {
      float rpm = parseRPMValue(command, 6);
      for (int i = 0; i < NMOTORS; i++) {
        enablePIDForMotor(i, rpm);
      }
      Serial.print("Set ALL motors to RPM=");
      Serial.println(rpm, 1);

    } else if (command.startsWith("RPMLEFT")) {
      float rpm = parseRPMValue(command, 7);
      for (int i = 0; i < 3; i++) {
        enablePIDForMotor(LEFT_MOTORS[i], rpm);
      }
      Serial.print("Set LEFT motors to RPM=");
      Serial.println(rpm, 1);

    } else if (command.startsWith("RPMRIGHT")) {
      float rpm = parseRPMValue(command, 8);
      for (int i = 0; i < 3; i++) {
        enablePIDForMotor(RIGHT_MOTORS[i], rpm);
      }
      Serial.print("Set RIGHT motors to RPM=");
      Serial.println(rpm, 1);

    } else if (command.startsWith("RPM")) {
      // RPM<n> <value>  e.g. "RPM0 30" or "RPM3 -20"
      // Find the motor index digit(s) and the RPM value
      int spacePos = command.indexOf(' ');
      if (spacePos > 3) {
        int motorIdx = command.substring(3, spacePos).toInt();
        float rpm = command.substring(spacePos + 1).toFloat();
        if (motorIdx >= 0 && motorIdx < NMOTORS) {
          enablePIDForMotor(motorIdx, rpm);
          Serial.print("Set M");
          Serial.print(motorIdx);
          Serial.print("(");
          Serial.print(MOTOR_LABELS[motorIdx]);
          Serial.print(") to RPM=");
          Serial.println(rpm, 1);
        }
      }

    // ---- TUNE commands ----
    } else if (command.startsWith("TUNEALL")) {
      // TUNEALL <Kp> <Ki> <Kd>
      String params = command.substring(7);
      params.trim();
      float kp, ki, kd;
      int s1 = params.indexOf(' ');
      int s2 = params.indexOf(' ', s1 + 1);
      if (s1 > 0 && s2 > s1) {
        kp = params.substring(0, s1).toFloat();
        ki = params.substring(s1 + 1, s2).toFloat();
        kd = params.substring(s2 + 1).toFloat();
        for (int i = 0; i < NMOTORS; i++) {
          pidKp[i] = kp;
          pidKi[i] = ki;
          pidKd[i] = kd;
          pidControllers[i]->SetTunings(kp, ki, kd);
        }
        Serial.print("Set ALL motors: Kp=");
        Serial.print(kp, 4);
        Serial.print(" Ki=");
        Serial.print(ki, 4);
        Serial.print(" Kd=");
        Serial.println(kd, 4);
      } else {
        Serial.println("Usage: TUNEALL <Kp> <Ki> <Kd>");
      }

    } else if (command.startsWith("TUNE")) {
      // TUNE<n> <Kp> <Ki> <Kd>
      int spacePos = command.indexOf(' ');
      if (spacePos > 4) {
        int motorIdx = command.substring(4, spacePos).toInt();
        String params = command.substring(spacePos + 1);
        params.trim();
        int s1 = params.indexOf(' ');
        int s2 = params.indexOf(' ', s1 + 1);
        if (motorIdx >= 0 && motorIdx < NMOTORS && s1 > 0 && s2 > s1) {
          float kp = params.substring(0, s1).toFloat();
          float ki = params.substring(s1 + 1, s2).toFloat();
          float kd = params.substring(s2 + 1).toFloat();
          pidKp[motorIdx] = kp;
          pidKi[motorIdx] = ki;
          pidKd[motorIdx] = kd;
          pidControllers[motorIdx]->SetTunings(kp, ki, kd);
          Serial.print("Set M");
          Serial.print(motorIdx);
          Serial.print(": Kp=");
          Serial.print(kp, 4);
          Serial.print(" Ki=");
          Serial.print(ki, 4);
          Serial.print(" Kd=");
          Serial.println(kd, 4);
        } else {
          Serial.println("Usage: TUNE<n> <Kp> <Ki> <Kd>");
        }
      }

    // ---- SAVE / LOAD ----
    } else if (command == "SAVE") {
      savePIDConfig();

    } else if (command == "LOAD") {
      loadPIDConfig();

    // ---- Manual motor commands (existing) ----
    } else if (command.startsWith("M")) {
      int motorIdx = -1;
      int pwm = 0;
      int dir = 1;
      int mPos = command.indexOf('M');
      int pPos = command.indexOf('P');
      int dPos = command.indexOf('D');

      if (mPos >= 0 && pPos > mPos) {
        motorIdx = command.substring(mPos + 1, pPos).toInt();
        pwm = command.substring(pPos + 1, dPos > pPos ? dPos : command.length()).toInt();
        if (dPos > pPos) {
          dir = command.substring(dPos + 1).toInt();
        }
        if (motorIdx >= 0 && motorIdx < NMOTORS) {
          disablePID(motorIdx);  // Switch to manual
          motorPWM[motorIdx] = pwm;
          motorDirection[motorIdx] = dir;
          isStopping[motorIdx] = false;
          Serial.print("Set Motor ");
          Serial.print(motorIdx);
          Serial.print(" to PWM=");
          Serial.print(pwm);
          Serial.print(" DIR=");
          Serial.print(dir);
          Serial.println(" [MANUAL]");
        }
      }

    } else if (command.startsWith("ALL")) {
      int pwm = 0;
      int dir = 1;
      int pPos = command.indexOf('P');
      int dPos = command.indexOf('D');
      if (pPos >= 0) {
        pwm = command.substring(pPos + 1, dPos > pPos ? dPos : command.length()).toInt();
        if (dPos > pPos) dir = command.substring(dPos + 1).toInt();
        for (int i = 0; i < NMOTORS; i++) {
          disablePID(i);
          motorPWM[i] = pwm;
          motorDirection[i] = dir;
          isStopping[i] = false;
        }
        Serial.print("Set ALL Motors to PWM=");
        Serial.print(pwm);
        Serial.print(" DIR=");
        Serial.print(dir);
        Serial.println(" [MANUAL]");
      }

    } else if (command.startsWith("LEFT")) {
      int pwm = 0;
      int dir = 1;
      int pPos = command.indexOf('P');
      int dPos = command.indexOf('D');
      if (pPos >= 0) {
        pwm = command.substring(pPos + 1, dPos > pPos ? dPos : command.length()).toInt();
        if (dPos > pPos) dir = command.substring(dPos + 1).toInt();
        for (int i = 0; i < 3; i++) {
          int idx = LEFT_MOTORS[i];
          disablePID(idx);
          motorPWM[idx] = pwm;
          motorDirection[idx] = dir;
          isStopping[idx] = false;
        }
        Serial.print("Set LEFT to PWM=");
        Serial.print(pwm);
        Serial.print(" DIR=");
        Serial.print(dir);
        Serial.println(" [MANUAL]");
      }

    } else if (command.startsWith("RIGHT")) {
      int pwm = 0;
      int dir = 1;
      int pPos = command.indexOf('P');
      int dPos = command.indexOf('D');
      if (pPos >= 0) {
        pwm = command.substring(pPos + 1, dPos > pPos ? dPos : command.length()).toInt();
        if (dPos > pPos) dir = command.substring(dPos + 1).toInt();
        for (int i = 0; i < 3; i++) {
          int idx = RIGHT_MOTORS[i];
          disablePID(idx);
          motorPWM[idx] = pwm;
          motorDirection[idx] = dir;
          isStopping[idx] = false;
        }
        Serial.print("Set RIGHT to PWM=");
        Serial.print(pwm);
        Serial.print(" DIR=");
        Serial.print(dir);
        Serial.println(" [MANUAL]");
      }

    } else if (command.startsWith("RAMPDOWN")) {
      startRampDown();

    } else if (command.startsWith("STOPALL")) {
      gentleStopAllMotors();
      Serial.println("Gentle stopping ALL motors...");

    } else if (command.startsWith("XALL")) {
      cancelAutoTune();
      for (int i = 0; i < NMOTORS; i++) {
        disablePID(i);
        motorPWM[i] = 0;
        isStopping[i] = false;
        stopMotor(i);
      }
      Serial.println("Immediately stopped ALL motors");

    } else if (command.startsWith("S") && command.length() >= 2 && isDigit(command.charAt(1))) {
      int motorIdx = command.substring(1).toInt();
      if (motorIdx >= 0 && motorIdx < NMOTORS) {
        gentleStopMotor(motorIdx);
        Serial.print("Gentle stopping Motor ");
        Serial.println(motorIdx);
      }

    } else if (command.startsWith("X") && command.length() >= 2 && isDigit(command.charAt(1))) {
      int motorIdx = command.substring(1).toInt();
      if (motorIdx >= 0 && motorIdx < NMOTORS) {
        disablePID(motorIdx);
        motorPWM[motorIdx] = 0;
        isStopping[motorIdx] = false;
        stopMotor(motorIdx);
        Serial.print("Immediately stopped Motor ");
        Serial.println(motorIdx);
      }

    } else {
      Serial.print("Unknown command: ");
      Serial.println(command);
    }
  }

  delay(1);  // Small delay
}

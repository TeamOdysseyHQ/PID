/**
 * @file BasicMotorControl.cpp
 * @brief 6-Wheel Differential Drive Motor Control with QuickPID for Teensy 4.1
 * 
 * Features:
 * - QuickPID control with TIMER mode (10ms update rate)
 * - Built-in AutoTuning with 10 methods (lbussy/QuickPID)
 * - EEPROM storage for PID parameters
 * - micro-ROS subscription to /teensy_motor_rpms (Float32MultiArray)
 * - micro-ROS publishing to /teensy_motor_feedback
 * 
 * Motor Mapping (CONFIGURABLE):
 * - RIGHT side motors: M0, M1, M2 (DIR pin 26)
 * - LEFT side motors: M3, M4, M5 (DIR pin 27)
 * 
 * micro-ROS Topic Format:
 * - /teensy_motor_rpms: [left0, right1, left2, right3, left4, right5]
 *   Mapping: left0->M3, right1->M0, left2->M4, right3->M1, left4->M5, right5->M2
 */

#include <Arduino.h>
#include <Encoder.h>
#include <QuickPID.h>
#include <EEPROM.h>

// ============================================================================
// CONFIGURATION - MOTOR SIDE MAPPING (EDITABLE)
// ============================================================================
// RIGHT side motors: M0, M1, M2 (indices into motor arrays)
// LEFT side motors: M3, M4, M5
const int RIGHT_MOTORS[] = {0, 1, 2};  // M0, M1, M2 are RIGHT wheels
const int LEFT_MOTORS[] = {3, 4, 5};   // M3, M4, M5 are LEFT wheels

// Mapping from Float32MultiArray [left0, right1, left2, right3, left4, right5] to motor index
// Array index = position in Float32MultiArray, value = motor index
// Format: [left0->M3, right1->M0, left2->M4, right3->M1, left4->M5, right5->M2]
const int ROS_TO_MOTOR_MAP[] = {3, 0, 4, 1, 5, 2};  // EDITABLE: Adjust mapping here

// ============================================================================
// SYSTEM CONFIGURATION
// ============================================================================
#define NMOTORS 6

// Operating modes
enum OperatingMode {
  MODE_IDLE,
  MODE_AUTOTUNE,
  MODE_PID_CONTROL,
  MODE_DIRECT_PWM
};
OperatingMode currentMode = MODE_IDLE;
int autotuneMotorIndex = -1;

// ============================================================================
// PIN DEFINITIONS - Teensy 4.1
// ============================================================================
const int ENCO_CH_1[] = {21, 39, 17, 37, 41, 35};  // Encoder Channel A
const int ENCO_CH_2[] = {20, 38, 16, 36, 40, 34};  // Encoder Channel B
const int PWM_PIN[] = {2, 3, 4, 5, 6, 7};          // PWM pins
const int DIR_PIN[] = {26, 26, 26, 27, 27, 27};    // Direction pins (M0-M2: 26, M3-M5: 27)

// ============================================================================
// ENCODER CONFIGURATION
// ============================================================================
const int PULSES_PER_REV = 7;
const int GEAR_RATIO[] = {181, 181, 181, 181, 181, 181};
// Counts per revolution for each motor (quadrature = x4)
const int COUNTS_PER_REV[] = {
  PULSES_PER_REV * GEAR_RATIO[0] * 4,  // Motor 0
  PULSES_PER_REV * GEAR_RATIO[1] * 4,  // Motor 1
  PULSES_PER_REV * GEAR_RATIO[2] * 4,  // Motor 2
  PULSES_PER_REV * GEAR_RATIO[3] * 4,  // Motor 3
  PULSES_PER_REV * GEAR_RATIO[4] * 4,  // Motor 4
  PULSES_PER_REV * GEAR_RATIO[5] * 4   // Motor 5
};

// Create Encoder objects
Encoder encoder0(ENCO_CH_1[0], ENCO_CH_2[0]);
Encoder encoder1(ENCO_CH_1[1], ENCO_CH_2[1]);
Encoder encoder2(ENCO_CH_1[2], ENCO_CH_2[2]);
Encoder encoder3(ENCO_CH_1[3], ENCO_CH_2[3]);
Encoder encoder4(ENCO_CH_1[4], ENCO_CH_2[4]);
Encoder encoder5(ENCO_CH_1[5], ENCO_CH_2[5]);

// ============================================================================
// PID CONFIGURATION
// ============================================================================
const unsigned long PID_SAMPLE_TIME_US = 10000;  // 10ms = 100Hz PID update rate

// PID variables for each motor
float pidInput[NMOTORS] = {0};       // Current RPM (from encoder)
float pidOutput[NMOTORS] = {0};      // PWM output (0-255)
float pidSetpoint[NMOTORS] = {0};    // Target RPM

// PID tuning parameters (will be loaded from EEPROM or defaults)
float Kp[NMOTORS] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
float Ki[NMOTORS] = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
float Kd[NMOTORS] = {0.05, 0.05, 0.05, 0.05, 0.05, 0.05};

// POn/DOn ratios (1.0 = on error, 0.0 = on measurement)
const float POn = 1.0;  // Proportional on Error
const float DOn = 0.0;  // Derivative on Measurement (reduces derivative kick)

// Create QuickPID objects
QuickPID motorPID[NMOTORS] = {
  QuickPID(&pidInput[0], &pidOutput[0], &pidSetpoint[0], Kp[0], Ki[0], Kd[0], POn, DOn, QuickPID::DIRECT),
  QuickPID(&pidInput[1], &pidOutput[1], &pidSetpoint[1], Kp[1], Ki[1], Kd[1], POn, DOn, QuickPID::DIRECT),
  QuickPID(&pidInput[2], &pidOutput[2], &pidSetpoint[2], Kp[2], Ki[2], Kd[2], POn, DOn, QuickPID::DIRECT),
  QuickPID(&pidInput[3], &pidOutput[3], &pidSetpoint[3], Kp[3], Ki[3], Kd[3], POn, DOn, QuickPID::DIRECT),
  QuickPID(&pidInput[4], &pidOutput[4], &pidSetpoint[4], Kp[4], Ki[4], Kd[4], POn, DOn, QuickPID::DIRECT),
  QuickPID(&pidInput[5], &pidOutput[5], &pidSetpoint[5], Kp[5], Ki[5], Kd[5], POn, DOn, QuickPID::DIRECT)
};

// ============================================================================
// EEPROM CONFIGURATION
// ============================================================================
const int EEPROM_MAGIC = 0xABCD;  // Magic number to verify valid data
const int EEPROM_MAGIC_ADDR = 0;
const int EEPROM_PID_START_ADDR = 4;
// Each motor stores: Kp (4 bytes) + Ki (4 bytes) + Kd (4 bytes) = 12 bytes
const int EEPROM_MOTOR_SIZE = 12;

// ============================================================================
// RPM CALCULATION VARIABLES
// ============================================================================
unsigned long prevTime[NMOTORS] = {0};
long prevEncoderPos[NMOTORS] = {0};
float motorRPM[NMOTORS] = {0};
float motorRPM_filtered[NMOTORS] = {0};
const float RPM_FILTER_ALPHA = 0.3;  // Low-pass filter coefficient

// ============================================================================
// MOTOR CONTROL VARIABLES
// ============================================================================
int motorPWM[NMOTORS] = {0};
int motorDirection[NMOTORS] = {1, 1, 1, 1, 1, 1};

// Gentle stop parameters
bool isStopping[NMOTORS] = {false};
int targetPWM[NMOTORS] = {0};
const int PWM_RAMP_STEP = 5;
const int PWM_RAMP_DELAY = 20;
unsigned long lastRampTime[NMOTORS] = {0};

// ============================================================================
// AUTOTUNE CONFIGURATION
// ============================================================================
const float AUTOTUNE_OUTPUT_STEP = 100;   // PWM step for autotuning
const float AUTOTUNE_HYSTERESIS = 5.0;    // RPM deadband
const float AUTOTUNE_SETPOINT = 50.0;     // Target RPM for autotuning
tuningMethod AUTOTUNE_METHOD = tuningMethod::ZIEGLER_NICHOLS_PID;

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================
long getEncoderPos(int motorIndex);
void setMotor(int motorIndex, int pwmValue, int direction);
void stopMotor(int motorIndex);
void gentleStopMotor(int motorIndex);
void gentleStopAllMotors();
void updatePWMRamp();
void calculateRPM();
void savePIDToEEPROM(int motorIndex);
void saveAllPIDToEEPROM();
void loadPIDFromEEPROM();
void printPIDValues();
void printHelp();
void processSerialCommand();
void runAutoTune(int motorIndex);
void updatePIDControl();

// ============================================================================
// ENCODER FUNCTIONS
// ============================================================================
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

void resetEncoder(int motorIndex) {
  switch (motorIndex) {
    case 0: encoder0.write(0); break;
    case 1: encoder1.write(0); break;
    case 2: encoder2.write(0); break;
    case 3: encoder3.write(0); break;
    case 4: encoder4.write(0); break;
    case 5: encoder5.write(0); break;
  }
}

// ============================================================================
// MOTOR CONTROL FUNCTIONS
// ============================================================================
void setMotor(int motorIndex, int pwmValue, int direction) {
  pwmValue = constrain(pwmValue, 0, 255);
  
  if (direction > 0) {
    digitalWrite(DIR_PIN[motorIndex], HIGH);
  } else {
    digitalWrite(DIR_PIN[motorIndex], LOW);
  }
  
  analogWrite(PWM_PIN[motorIndex], pwmValue);
}

void stopMotor(int motorIndex) {
  analogWrite(PWM_PIN[motorIndex], 0);
  motorPWM[motorIndex] = 0;
}

void gentleStopMotor(int motorIndex) {
  isStopping[motorIndex] = true;
  targetPWM[motorIndex] = 0;
}

void gentleStopAllMotors() {
  for (int i = 0; i < NMOTORS; i++) {
    gentleStopMotor(i);
  }
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

// ============================================================================
// RPM CALCULATION
// ============================================================================
void calculateRPM() {
  for (int i = 0; i < NMOTORS; i++) {
    unsigned long currentTime = micros();
    float deltaTime = ((float)(currentTime - prevTime[i])) / 1.0e6;
    
    if (deltaTime >= 0.01) {  // Calculate every 10ms for PID compatibility
      long currentPos = getEncoderPos(i);
      long deltaPos = currentPos - prevEncoderPos[i];
      
      // Calculate raw RPM
      float rawRPM = (deltaPos / deltaTime) / COUNTS_PER_REV[i] * 60.0;
      
      // Low-pass filter
      motorRPM_filtered[i] = RPM_FILTER_ALPHA * rawRPM + (1.0 - RPM_FILTER_ALPHA) * motorRPM_filtered[i];
      motorRPM[i] = motorRPM_filtered[i];
      
      prevEncoderPos[i] = currentPos;
      prevTime[i] = currentTime;
    }
  }
}

// ============================================================================
// EEPROM FUNCTIONS
// ============================================================================
void savePIDToEEPROM(int motorIndex) {
  if (motorIndex < 0 || motorIndex >= NMOTORS) return;
  
  int addr = EEPROM_PID_START_ADDR + (motorIndex * EEPROM_MOTOR_SIZE);
  EEPROM.put(addr, Kp[motorIndex]);
  EEPROM.put(addr + 4, Ki[motorIndex]);
  EEPROM.put(addr + 8, Kd[motorIndex]);
  
  // Write magic number to indicate valid data
  EEPROM.put(EEPROM_MAGIC_ADDR, EEPROM_MAGIC);
  
  Serial.print("Saved Motor ");
  Serial.print(motorIndex);
  Serial.print(" PID to EEPROM: Kp=");
  Serial.print(Kp[motorIndex], 4);
  Serial.print(" Ki=");
  Serial.print(Ki[motorIndex], 4);
  Serial.print(" Kd=");
  Serial.println(Kd[motorIndex], 4);
}

void saveAllPIDToEEPROM() {
  for (int i = 0; i < NMOTORS; i++) {
    int addr = EEPROM_PID_START_ADDR + (i * EEPROM_MOTOR_SIZE);
    EEPROM.put(addr, Kp[i]);
    EEPROM.put(addr + 4, Ki[i]);
    EEPROM.put(addr + 8, Kd[i]);
  }
  EEPROM.put(EEPROM_MAGIC_ADDR, EEPROM_MAGIC);
  Serial.println("All PID values saved to EEPROM");
}

void loadPIDFromEEPROM() {
  int magic;
  EEPROM.get(EEPROM_MAGIC_ADDR, magic);
  
  if (magic != EEPROM_MAGIC) {
    Serial.println("No valid PID data in EEPROM, using defaults");
    return;
  }
  
  Serial.println("Loading PID values from EEPROM:");
  for (int i = 0; i < NMOTORS; i++) {
    int addr = EEPROM_PID_START_ADDR + (i * EEPROM_MOTOR_SIZE);
    EEPROM.get(addr, Kp[i]);
    EEPROM.get(addr + 4, Ki[i]);
    EEPROM.get(addr + 8, Kd[i]);
    
    // Apply loaded values to PID controllers
    motorPID[i].SetTunings(Kp[i], Ki[i], Kd[i]);
    
    Serial.print("  Motor ");
    Serial.print(i);
    Serial.print(": Kp=");
    Serial.print(Kp[i], 4);
    Serial.print(" Ki=");
    Serial.print(Ki[i], 4);
    Serial.print(" Kd=");
    Serial.println(Kd[i], 4);
  }
}

void clearEEPROM() {
  EEPROM.put(EEPROM_MAGIC_ADDR, 0);
  Serial.println("EEPROM cleared");
}

void printPIDValues() {
  Serial.println("\n========== CURRENT PID VALUES ==========");
  Serial.println("Motor Side Mapping:");
  Serial.print("  RIGHT motors: M");
  for (int i = 0; i < 3; i++) {
    Serial.print(RIGHT_MOTORS[i]);
    if (i < 2) Serial.print(", M");
  }
  Serial.println();
  Serial.print("  LEFT motors: M");
  for (int i = 0; i < 3; i++) {
    Serial.print(LEFT_MOTORS[i]);
    if (i < 2) Serial.print(", M");
  }
  Serial.println("\n");
  
  Serial.println("PID Parameters:");
  for (int i = 0; i < NMOTORS; i++) {
    String side = "UNKNOWN";
    for (int j = 0; j < 3; j++) {
      if (RIGHT_MOTORS[j] == i) { side = "RIGHT"; break; }
      if (LEFT_MOTORS[j] == i) { side = "LEFT"; break; }
    }
    
    Serial.print("  Motor ");
    Serial.print(i);
    Serial.print(" (");
    Serial.print(side);
    Serial.print("): Kp=");
    Serial.print(Kp[i], 4);
    Serial.print(" Ki=");
    Serial.print(Ki[i], 4);
    Serial.print(" Kd=");
    Serial.println(Kd[i], 4);
  }
  Serial.println("==========================================\n");
}

// ============================================================================
// AUTOTUNE FUNCTIONS
// ============================================================================
void startAutoTune(int motorIndex) {
  if (motorIndex < 0 || motorIndex >= NMOTORS) {
    Serial.println("Invalid motor index");
    return;
  }
  
  Serial.print("\n========== STARTING AUTOTUNE FOR MOTOR ");
  Serial.print(motorIndex);
  Serial.println(" ==========");
  Serial.println("Keep motor unloaded and in free-running position");
  Serial.println("AutoTune will complete in approximately 10-30 seconds");
  Serial.println("Press 'C' to cancel\n");
  
  // Stop all other motors
  for (int i = 0; i < NMOTORS; i++) {
    if (i != motorIndex) {
      stopMotor(i);
      motorPID[i].SetMode(QuickPID::MANUAL);
    }
  }
  
  // Reset encoder for clean start
  resetEncoder(motorIndex);
  prevEncoderPos[motorIndex] = 0;
  pidInput[motorIndex] = 0;
  
  // Initialize AutoTune
  motorPID[motorIndex].AutoTune(AUTOTUNE_METHOD);
  motorPID[motorIndex].autoTune->autoTuneConfig(
    AUTOTUNE_OUTPUT_STEP,    // Output step (PWM)
    AUTOTUNE_HYSTERESIS,     // Hysteresis (RPM deadband)
    AUTOTUNE_SETPOINT,       // Setpoint (target RPM)
    pidOutput[motorIndex],   // Initial output
    QuickPID::DIRECT,        // Direction
    true,                    // Print to Serial
    PID_SAMPLE_TIME_US       // Sample time
  );
  
  autotuneMotorIndex = motorIndex;
  currentMode = MODE_AUTOTUNE;
}

void cancelAutoTune() {
  if (autotuneMotorIndex >= 0 && autotuneMotorIndex < NMOTORS) {
    motorPID[autotuneMotorIndex].clearAutoTune();
    stopMotor(autotuneMotorIndex);
    Serial.println("AutoTune cancelled");
  }
  autotuneMotorIndex = -1;
  currentMode = MODE_IDLE;
}

void runAutoTuneLoop() {
  if (autotuneMotorIndex < 0 || autotuneMotorIndex >= NMOTORS) return;
  
  int i = autotuneMotorIndex;
  
  // Update RPM reading
  calculateRPM();
  pidInput[i] = abs(motorRPM[i]);  // Use absolute RPM
  
  // Run AutoTune step
  if (motorPID[i].autoTune) {
    byte tuneResult = motorPID[i].autoTune->autoTuneLoop();
    
    switch (tuneResult) {
      case motorPID[i].autoTune->AUTOTUNE:
        // Still tuning - apply output to motor
        setMotor(i, (int)pidOutput[i], 1);
        break;
        
      case motorPID[i].autoTune->TUNINGS:
        // Tuning complete - get constants
        motorPID[i].autoTune->setAutoTuneConstants(&Kp[i], &Ki[i], &Kd[i]);
        
        Serial.println("\n========== AUTOTUNE COMPLETE ==========");
        Serial.print("Motor ");
        Serial.print(i);
        Serial.println(" tuned successfully!");
        Serial.print("Kp = ");
        Serial.println(Kp[i], 4);
        Serial.print("Ki = ");
        Serial.println(Ki[i], 4);
        Serial.print("Kd = ");
        Serial.println(Kd[i], 4);
        Serial.println("\nCopy these values for backup:");
        Serial.print("Motor ");
        Serial.print(i);
        Serial.print(": Kp=");
        Serial.print(Kp[i], 6);
        Serial.print(", Ki=");
        Serial.print(Ki[i], 6);
        Serial.print(", Kd=");
        Serial.println(Kd[i], 6);
        
        // Apply tunings to PID controller
        motorPID[i].SetTunings(Kp[i], Ki[i], Kd[i]);
        
        // Save to EEPROM
        savePIDToEEPROM(i);
        
        Serial.println("=========================================\n");
        break;
        
      case motorPID[i].autoTune->CLR:
        // Clean up
        motorPID[i].clearAutoTune();
        stopMotor(i);
        autotuneMotorIndex = -1;
        currentMode = MODE_IDLE;
        Serial.println("AutoTune cleanup complete. Ready for next motor.");
        break;
    }
  }
}

// ============================================================================
// PID CONTROL FUNCTIONS
// ============================================================================
void initializePID() {
  for (int i = 0; i < NMOTORS; i++) {
    motorPID[i].SetOutputLimits(0, 255);
    motorPID[i].SetSampleTimeUs(PID_SAMPLE_TIME_US);
    motorPID[i].SetMode(QuickPID::TIMER);  // TIMER mode for ISR compatibility
  }
}

void updatePIDControl() {
  if (currentMode != MODE_PID_CONTROL) return;
  
  for (int i = 0; i < NMOTORS; i++) {
    // Update input from encoder
    pidInput[i] = abs(motorRPM[i]);
    
    // Compute PID
    motorPID[i].Compute();
    
    // Determine direction based on setpoint sign
    int direction = (pidSetpoint[i] >= 0) ? 1 : -1;
    
    // Apply output to motor
    setMotor(i, (int)pidOutput[i], direction);
    motorPWM[i] = (int)pidOutput[i];
    motorDirection[i] = direction;
  }
}

void startPIDControl() {
  currentMode = MODE_PID_CONTROL;
  for (int i = 0; i < NMOTORS; i++) {
    motorPID[i].SetMode(QuickPID::TIMER);
  }
  Serial.println("PID Control mode activated");
}

void stopPIDControl() {
  currentMode = MODE_IDLE;
  for (int i = 0; i < NMOTORS; i++) {
    motorPID[i].SetMode(QuickPID::MANUAL);
    stopMotor(i);
  }
  Serial.println("PID Control stopped");
}

// ============================================================================
// SERIAL COMMAND INTERFACE
// ============================================================================
void printHelp() {
  Serial.println("\n============ COMMAND REFERENCE ============");
  Serial.println("MOTOR CONTROL:");
  Serial.println("  M<n> P<pwm> D<dir>  - Direct PWM control (Motor n, PWM 0-255, Dir 1/-1)");
  Serial.println("  ALL P<pwm> D<dir>   - Set all motors to same PWM");
  Serial.println("  S<n>                - Gentle stop motor n");
  Serial.println("  X<n>                - Immediate stop motor n");
  Serial.println("  STOPALL             - Gentle stop all");
  Serial.println("  XALL                - Immediate stop all");
  Serial.println("");
  Serial.println("PID CONTROL:");
  Serial.println("  PID ON              - Start PID control mode");
  Serial.println("  PID OFF             - Stop PID control mode");
  Serial.println("  RPM <n> <value>     - Set target RPM for motor n");
  Serial.println("  RPMALL <value>      - Set target RPM for all motors");
  Serial.println("  TUNE <n> Kp Ki Kd   - Manually set PID values for motor n");
  Serial.println("");
  Serial.println("AUTOTUNING:");
  Serial.println("  AT <n>              - Start AutoTune for motor n (0-5)");
  Serial.println("  ATALL               - AutoTune all motors sequentially");
  Serial.println("  C                   - Cancel AutoTune");
  Serial.println("  METHOD <n>          - Set AutoTune method (0-9)");
  Serial.println("    0=ZN_PI, 1=ZN_PID, 2=TL_PI, 3=TL_PID, 4=CM_PI,");
  Serial.println("    5=CM_PID, 6=AMIGOF, 7=PESSEN, 8=SOME_OS, 9=NO_OS");
  Serial.println("");
  Serial.println("EEPROM:");
  Serial.println("  SAVE                - Save all PID values to EEPROM");
  Serial.println("  LOAD                - Load PID values from EEPROM");
  Serial.println("  CLEAR               - Clear EEPROM");
  Serial.println("");
  Serial.println("INFO:");
  Serial.println("  SHOW                - Show current PID values");
  Serial.println("  STATUS              - Show system status");
  Serial.println("  HELP                - Show this help");
  Serial.println("=============================================\n");
}

void printStatus() {
  Serial.println("\n============ SYSTEM STATUS ============");
  Serial.print("Operating Mode: ");
  switch (currentMode) {
    case MODE_IDLE: Serial.println("IDLE"); break;
    case MODE_AUTOTUNE: Serial.print("AUTOTUNING Motor "); Serial.println(autotuneMotorIndex); break;
    case MODE_PID_CONTROL: Serial.println("PID CONTROL"); break;
    case MODE_DIRECT_PWM: Serial.println("DIRECT PWM"); break;
  }
  Serial.println("\nMotor States:");
  for (int i = 0; i < NMOTORS; i++) {
    Serial.print("  M");
    Serial.print(i);
    Serial.print(": RPM=");
    Serial.print(motorRPM[i], 1);
    Serial.print(" PWM=");
    Serial.print(motorPWM[i]);
    Serial.print(" Target=");
    Serial.print(pidSetpoint[i], 1);
    Serial.print(" Pos=");
    Serial.println(getEncoderPos(i));
  }
  Serial.println("=========================================\n");
}

void processSerialCommand() {
  if (!Serial.available()) return;
  
  String command = Serial.readStringUntil('\n');
  command.trim();
  command.toUpperCase();
  
  // Check for cancel during autotune
  if (currentMode == MODE_AUTOTUNE && command == "C") {
    cancelAutoTune();
    return;
  }
  
  // Parse commands
  if (command.startsWith("AT ") && !command.startsWith("ATALL")) {
    // AutoTune single motor: AT <n>
    int motorIdx = command.substring(3).toInt();
    startAutoTune(motorIdx);
  }
  else if (command == "ATALL") {
    // AutoTune all motors - start with motor 0
    Serial.println("AutoTune ALL motors - starting with Motor 0");
    Serial.println("Each motor will be tuned sequentially");
    startAutoTune(0);
  }
  else if (command.startsWith("METHOD ")) {
    int method = command.substring(7).toInt();
    if (method >= 0 && method <= 9) {
      AUTOTUNE_METHOD = (tuningMethod)method;
      Serial.print("AutoTune method set to: ");
      Serial.println(method);
    }
  }
  else if (command == "PID ON") {
    startPIDControl();
  }
  else if (command == "PID OFF") {
    stopPIDControl();
  }
  else if (command.startsWith("RPM ")) {
    // RPM <n> <value>
    int space = command.indexOf(' ', 4);
    if (space > 0) {
      int motorIdx = command.substring(4, space).toInt();
      float rpm = command.substring(space + 1).toFloat();
      if (motorIdx >= 0 && motorIdx < NMOTORS) {
        pidSetpoint[motorIdx] = rpm;
        Serial.print("Motor ");
        Serial.print(motorIdx);
        Serial.print(" target RPM set to: ");
        Serial.println(rpm);
      }
    }
  }
  else if (command.startsWith("RPMALL ")) {
    float rpm = command.substring(7).toFloat();
    for (int i = 0; i < NMOTORS; i++) {
      pidSetpoint[i] = rpm;
    }
    Serial.print("All motors target RPM set to: ");
    Serial.println(rpm);
  }
  else if (command.startsWith("TUNE ")) {
    // TUNE <n> Kp Ki Kd
    int idx1 = 5;
    int idx2 = command.indexOf(' ', idx1);
    int idx3 = command.indexOf(' ', idx2 + 1);
    int idx4 = command.indexOf(' ', idx3 + 1);
    
    if (idx2 > 0 && idx3 > 0 && idx4 > 0) {
      int motorIdx = command.substring(idx1, idx2).toInt();
      float kp = command.substring(idx2 + 1, idx3).toFloat();
      float ki = command.substring(idx3 + 1, idx4).toFloat();
      float kd = command.substring(idx4 + 1).toFloat();
      
      if (motorIdx >= 0 && motorIdx < NMOTORS) {
        Kp[motorIdx] = kp;
        Ki[motorIdx] = ki;
        Kd[motorIdx] = kd;
        motorPID[motorIdx].SetTunings(kp, ki, kd);
        
        Serial.print("Motor ");
        Serial.print(motorIdx);
        Serial.print(" PID set to: Kp=");
        Serial.print(kp, 4);
        Serial.print(" Ki=");
        Serial.print(ki, 4);
        Serial.print(" Kd=");
        Serial.println(kd, 4);
      }
    }
  }
  else if (command == "SAVE") {
    saveAllPIDToEEPROM();
    printPIDValues();
  }
  else if (command == "LOAD") {
    loadPIDFromEEPROM();
  }
  else if (command == "CLEAR") {
    clearEEPROM();
  }
  else if (command == "SHOW") {
    printPIDValues();
  }
  else if (command == "STATUS") {
    printStatus();
  }
  else if (command == "HELP") {
    printHelp();
  }
  else if (command.startsWith("M") && command.indexOf('P') > 0) {
    // Direct motor control: M<n> P<pwm> D<dir>
    currentMode = MODE_DIRECT_PWM;
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
        motorPWM[motorIdx] = pwm;
        motorDirection[motorIdx] = dir;
        isStopping[motorIdx] = false;
        setMotor(motorIdx, pwm, dir);
        Serial.print("Set Motor ");
        Serial.print(motorIdx);
        Serial.print(" to PWM=");
        Serial.print(pwm);
        Serial.print(" DIR=");
        Serial.println(dir);
      }
    }
  }
  else if (command.startsWith("ALL P")) {
    currentMode = MODE_DIRECT_PWM;
    int pwm = 0;
    int dir = 1;
    
    int pPos = command.indexOf('P');
    int dPos = command.indexOf('D');
    
    if (pPos >= 0) {
      pwm = command.substring(pPos + 1, dPos > pPos ? dPos : command.length()).toInt();
      if (dPos > pPos) {
        dir = command.substring(dPos + 1).toInt();
      }
      
      for (int i = 0; i < NMOTORS; i++) {
        motorPWM[i] = pwm;
        motorDirection[i] = dir;
        isStopping[i] = false;
        setMotor(i, pwm, dir);
      }
      Serial.print("Set ALL Motors to PWM=");
      Serial.print(pwm);
      Serial.print(" DIR=");
      Serial.println(dir);
    }
  }
  else if (command == "STOPALL") {
    gentleStopAllMotors();
    Serial.println("Gentle stopping ALL motors...");
  }
  else if (command == "XALL") {
    for (int i = 0; i < NMOTORS; i++) {
      motorPWM[i] = 0;
      isStopping[i] = false;
      stopMotor(i);
    }
    currentMode = MODE_IDLE;
    Serial.println("Immediately stopped ALL motors");
  }
  else if (command.startsWith("S") && command.length() == 2) {
    int motorIdx = command.substring(1).toInt();
    if (motorIdx >= 0 && motorIdx < NMOTORS) {
      gentleStopMotor(motorIdx);
      Serial.print("Gentle stopping Motor ");
      Serial.println(motorIdx);
    }
  }
  else if (command.startsWith("X") && command.length() == 2) {
    int motorIdx = command.substring(1).toInt();
    if (motorIdx >= 0 && motorIdx < NMOTORS) {
      motorPWM[motorIdx] = 0;
      isStopping[motorIdx] = false;
      stopMotor(motorIdx);
      Serial.print("Immediately stopped Motor ");
      Serial.println(motorIdx);
    }
  }
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(2000);
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  // Initialize motor pins
  for (int i = 0; i < NMOTORS; i++) {
    pinMode(ENCO_CH_1[i], INPUT);
    pinMode(ENCO_CH_2[i], INPUT);
    pinMode(PWM_PIN[i], OUTPUT);
    pinMode(DIR_PIN[i], OUTPUT);
    stopMotor(i);
    prevTime[i] = micros();
  }
  
  // Initialize PID controllers
  initializePID();
  
  // Load PID values from EEPROM
  loadPIDFromEEPROM();
  
  Serial.println("\n================================================");
  Serial.println("  6-WHEEL DIFFERENTIAL DRIVE MOTOR CONTROLLER");
  Serial.println("  Teensy 4.1 + QuickPID (lbussy/QuickPID)");
  Serial.println("================================================");
  Serial.println("\nMotor Mapping:");
  Serial.println("  RIGHT wheels: M0, M1, M2");
  Serial.println("  LEFT wheels:  M3, M4, M5");
  Serial.println("\nType HELP for command reference");
  Serial.println("Type SHOW to display current PID values\n");
  
  printPIDValues();
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  // Always calculate RPM
  calculateRPM();
  
  // Mode-specific operations
  switch (currentMode) {
    case MODE_AUTOTUNE:
      runAutoTuneLoop();
      break;
      
    case MODE_PID_CONTROL:
      updatePIDControl();
      break;
      
    case MODE_DIRECT_PWM:
      updatePWMRamp();
      for (int i = 0; i < NMOTORS; i++) {
        setMotor(i, motorPWM[i], motorDirection[i]);
      }
      break;
      
    case MODE_IDLE:
    default:
      updatePWMRamp();
      break;
  }
  
  // Process serial commands
  processSerialCommand();
  
  // Print status every 500ms (except during autotune)
  static unsigned long lastPrintTime = 0;
  if (currentMode != MODE_AUTOTUNE && millis() - lastPrintTime >= 500) {
    for (int i = 0; i < NMOTORS; i++) {
      Serial.print("M");
      Serial.print(i);
      Serial.print(": RPM=");
      Serial.print(motorRPM[i], 1);
      Serial.print(" PWM=");
      Serial.print(motorPWM[i]);
      Serial.print(" Pos=");
      Serial.print(getEncoderPos(i));
      if (currentMode == MODE_PID_CONTROL) {
        Serial.print(" Target=");
        Serial.print(pidSetpoint[i], 1);
      }
      Serial.println();
    }
    Serial.println();
    lastPrintTime = millis();
  }
  
  delay(1); 
} 

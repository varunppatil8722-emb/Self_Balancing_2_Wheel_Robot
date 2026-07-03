// =============================================================================
//  SELF-BALANCING ROBOT — Arduino Mega 2560 + MPU6050 + 2×NEMA17 + 2×A4988
//  Author  : Generated for production use (Bug Fix & Performance Optimized)
//  Version : 2.1  (stability-optimised build)
// =============================================================================
//
//  PIN MAP (fixed — do not change)
//  ─────────────────────────────────────────────────────────────────────────
//  D4  → ENABLE  (active-LOW on A4988; HIGH = drivers off)
//  D5  → STEP1   (left  motor)
//  D6  → STEP2   (right motor)
//  D7  → DIR1    (left  motor)
//  D8  → DIR2    (right motor)
//  20  → SDA     (MPU6050 I²C data)
//  21  → SCL     (MPU6050 I²C clock)
//
//  MICROSTEPPING OPTIMIZATION (RECOMMENDED)
//  ─────────────────────────────────────────────────────────────────────────
//  Set A4988 MS1=HIGH, MS2=HIGH, MS3=LOW on both drivers → 1/8 microstepping.
//  This gives 1600 steps/rev, reducing CPU processing overhead on Mega 2560
//  while maintaining excellent torque and smoothness.
//
// =============================================================================

#include <Wire.h>
#include <AccelStepper.h>

// ─────────────────────────────────────────────────────────────────────────────
//  HARDWARE CONSTANTS
// ─────────────────────────────────────────────────────────────────────────────

// Pin definitions
#define PIN_ENABLE  4
#define PIN_STEP1   5
#define PIN_STEP2   6
#define PIN_DIR1    7
#define PIN_DIR2    8

// MPU6050 I²C address (AD0 low = 0x68, AD0 high = 0x69)
#define MPU_ADDR    0x68

// Microstepping: Updated to 1/8 to prevent step-loss (100 × 16 -> 200 × 8 = 1600)
#define STEPS_PER_REV  1600.0f

// ─────────────────────────────────────────────────────────────────────────────
//  TUNABLE PARAMETERS — adjust these to tune your robot
// ─────────────────────────────────────────────────────────────────────────────

// --- PID gains (Tune Kd slowly to damp the 7-10 second oscillations) ---
float Kp = 220.0f;   // Proportional: main restoring force
float Ki =   0.3f;   // Integral:     eliminates steady lean
float Kd =   2.5f;   // Derivative:   damps oscillation (increased slightly from 0)

// --- Complementary filter coefficient ---
#define COMP_FILTER_ALPHA  0.995f

// --- Derivative low-pass filter coefficient ---
#define DERIV_ALPHA        0.15f

// --- Control loop period ---
#define LOOP_PERIOD_US     5000UL   // 10 ms = 100 Hz

// --- Safety: shut down if tilt exceeds this angle (degrees) ---
#define FALL_ANGLE         35.0f

// --- Recovery: re-enable only when tilt returns within this angle ---
#define RECOVERY_ANGLE     10.0f

// --- Max PID output mapped to stepper speed (steps/sec) ---
// Adjusted for 1/8 microstepping to maintain same physical top speed limit
#define MAX_SPEED          2000.0f   // steps/sec
#define MAX_ACCEL          4000.0f   // steps/sec²

// --- Integral anti-windup clamp (in PID output units) ---
#define INTEGRAL_CLAMP     (MAX_SPEED * 0.4f)

// --- Balance angle offset (degrees) ---
#define BALANCE_ANGLE_OFFSET  0.0f

// --- Calibration ---
#define CALIB_SAMPLES      1000     
#define CALIB_SAMPLE_DELAY 3        

// --- Right motor direction inversion ---
#define INVERT_MOTOR1      true

// ─────────────────────────────────────────────────────────────────────────────
//  MPU6050 REGISTER ADDRESSES
// ─────────────────────────────────────────────────────────────────────────────
#define MPU_PWR_MGMT_1     0x6B
#define MPU_SMPLRT_DIV     0x19
#define MPU_CONFIG         0x1A
#define MPU_GYRO_CONFIG    0x1B
#define MPU_ACCEL_CONFIG   0x1C
#define MPU_ACCEL_XOUT_H   0x3B
#define MPU_GYRO_XOUT_H    0x43

#define ACCEL_SCALE  16384.0f
#define GYRO_SCALE     131.0f

// ─────────────────────────────────────────────────────────────────────────────
//  SENSOR DATA TYPE
// ─────────────────────────────────────────────────────────────────────────────
struct SensorData {
  float accelY;   
  float accelZ;   
  float gyroX;    
};

// ─────────────────────────────────────────────────────────────────────────────
//  GLOBAL STATE
// ─────────────────────────────────────────────────────────────────────────────

// AccelStepper objects
AccelStepper motor1(AccelStepper::DRIVER, PIN_STEP1, PIN_DIR1);
AccelStepper motor2(AccelStepper::DRIVER, PIN_STEP2, PIN_DIR2);

// Calibration offsets
float gyroX_offset  = 0.0f;
float accelY_offset = 0.0f;
float accelZ_offset = 0.0f;

// The measured upright angle during calibration
float balanceSetpoint = 0.0f;

// Complementary filter state
float filteredAngle = 0.0f;

// PID state
float integral          = 0.0f;
float prevMeasurement   = 0.0f;   // FIXED: Correct tracking variable name for derivative
float prevDerivative    = 0.0f;   // filtered derivative state

float lastAngle=0.0f;
float velocityDamping=0.15f;


// Timing
unsigned long prevTime = 0;

// Fallen / recovery state machine
bool isFallen = false;

// ─────────────────────────────────────────────────────────────────────────────
//  UTILITY: I²C WRITE / READ HELPERS
// ─────────────────────────────────────────────────────────────────────────────
void mpuWrite(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void mpuReadWords(uint8_t reg, float* out, uint8_t count) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)(count * 2), (uint8_t)true);
  for (uint8_t i = 0; i < count; i++) {
    int16_t raw = ((int16_t)Wire.read() << 8) | Wire.read();
    out[i] = (float)raw;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  MPU6050 INITIALISATION
// ─────────────────────────────────────────────────────────────────────────────
void initMPU() {
  mpuWrite(MPU_PWR_MGMT_1, 0x00);
  delay(150);  
  mpuWrite(MPU_SMPLRT_DIV, 0x09);
  mpuWrite(MPU_CONFIG, 0x03);
  mpuWrite(MPU_GYRO_CONFIG, 0x00);
  mpuWrite(MPU_ACCEL_CONFIG, 0x00);
}

// ─────────────────────────────────────────────────────────────────────────────
//  READ SENSOR DATA
// ─────────────────────────────────────────────────────────────────────────────
SensorData readSensor() {
  float raw[7];
  mpuReadWords(MPU_ACCEL_XOUT_H, raw, 7);

  SensorData s;
  s.accelY = (raw[1] - accelY_offset) / ACCEL_SCALE;
  s.accelZ = (raw[2] - accelZ_offset) / ACCEL_SCALE;
  s.gyroX  = (raw[4] - gyroX_offset)  / GYRO_SCALE;
  return s;
}

// ─────────────────────────────────────────────────────────────────────────────
//  STARTUP CALIBRATION
// ─────────────────────────────────────────────────────────────────────────────
void calibrate() {
  Serial.println(F("=== CALIBRATION STARTED ==="));
  Serial.println(F("Keep robot STATIONARY and UPRIGHT."));
  
  for (int i = 0; i < 100; i++) {
    float dummy[7];
    mpuReadWords(MPU_ACCEL_XOUT_H, dummy, 7);
    delay(CALIB_SAMPLE_DELAY);
  }

  Serial.println(F("Collecting calibration samples..."));

  double sumGyroX  = 0.0;
  double sumAccelY = 0.0;
  double sumAccelZ = 0.0;

  for (int i = 0; i < CALIB_SAMPLES; i++) {
    float raw[7];
    mpuReadWords(MPU_ACCEL_XOUT_H, raw, 7);

    sumGyroX  += raw[4];  
    sumAccelY += raw[1];  
    sumAccelZ += raw[2];  

    delay(CALIB_SAMPLE_DELAY);  

    if (i % 100 == 0) {  
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));  
      Serial.print('.');  
    }
  }
  Serial.println();

  gyroX_offset = (float)(sumGyroX / CALIB_SAMPLES);
  accelY_offset = (float)(sumAccelY / CALIB_SAMPLES);
  accelZ_offset = (float)(sumAccelZ / CALIB_SAMPLES) - ACCEL_SCALE;

  float ay = (float)(sumAccelY / CALIB_SAMPLES - accelY_offset) / ACCEL_SCALE;
  float az = (float)(sumAccelZ / CALIB_SAMPLES - accelZ_offset) / ACCEL_SCALE;
  balanceSetpoint = atan2f(ay, az) * RAD_TO_DEG + BALANCE_ANGLE_OFFSET;

  filteredAngle = balanceSetpoint;
  prevMeasurement = balanceSetpoint; // Initialize derivative tracker

  Serial.print(F("Gyro  offset (LSB): ")); Serial.println(gyroX_offset,  4);
  Serial.print(F("AccelY offset (LSB): ")); Serial.println(accelY_offset, 4);
  Serial.print(F("AccelZ offset (LSB): ")); Serial.println(accelZ_offset, 4);
  Serial.print(F("Balance setpoint (deg): ")); Serial.println(balanceSetpoint, 4);
  Serial.println(F("=== CALIBRATION COMPLETE ==="));
  delay(2000);
}

void enableMotors() {
  digitalWrite(PIN_ENABLE, LOW);   
}

void disableMotors() {
  digitalWrite(PIN_ENABLE, HIGH);  
  motor1.setSpeed(0);
  motor2.setSpeed(0);
}

void setMotorSpeed(float speed) {
  speed = constrain(speed, -MAX_SPEED, MAX_SPEED);
  motor1.setSpeed(INVERT_MOTOR1 ? -speed : speed);
  motor2.setSpeed(speed);
}

float complementaryFilter(float gyroRate, float accelY, float accelZ, float dt) {
  float gyroAngle  = filteredAngle + gyroRate * dt;
  float accelAngle = atan2f(accelY, accelZ) * RAD_TO_DEG;
  filteredAngle = COMP_FILTER_ALPHA * gyroAngle + (1.0f - COMP_FILTER_ALPHA) * accelAngle;
  return filteredAngle;
}

// ─────────────────────────────────────────────────────────────────────────────
//  FIXED PID CONTROLLER
// ─────────────────────────────────────────────────────────────────────────────
float pidUpdate(float setpoint, float measurement, float dt) {
  float error = setpoint - measurement;

  // --- Proportional ---
  float P = Kp * error;

  // --- Integral with anti-windup ---
  float newIntegral = integral + Ki * error * dt;
  newIntegral = constrain(newIntegral, -INTEGRAL_CLAMP, INTEGRAL_CLAMP);

  // --- Derivative on Measurement (FIXED BUG) ---
  // d(error)/dt = -d(measurement)/dt. Formula handles measurement changes accurately.
  static float prevMeasurement=0;
  float dRaw = -(measurement - prevMeasurement) / dt;   
   prevMeasurement= measurement; 
  // First-order IIR low-pass filter for D-term noise removal
  float D = DERIV_ALPHA * dRaw + (1.0f - DERIV_ALPHA) * prevDerivative;

  // Total control output
  float output = P + newIntegral + Kd * D;

  // Anti-windup execution
  bool saturated = (output >= MAX_SPEED && error > 0) || (output <= -MAX_SPEED && error < 0);
  if (!saturated) {
    integral = newIntegral;
  }

  // Update states for next execution loop
    // Fixed: Correct variable updated here
  prevDerivative  = D;

  return constrain(output, -MAX_SPEED, MAX_SPEED);
}

// ─────────────────────────────────────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println(F("\n============================"));
  Serial.println(F(" SELF-BALANCING ROBOT v2.1"));
  Serial.println(F("============================\n"));

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_ENABLE, OUTPUT);
  disableMotors();   

  motor1.setMaxSpeed(MAX_SPEED);
  motor2.setMaxSpeed(MAX_SPEED);
  motor1.setAcceleration(MAX_ACCEL);
  motor2.setAcceleration(MAX_ACCEL);

  Wire.begin();
  Wire.setClock(400000UL);
  delay(100);

  initMPU();
  calibrate();
  enableMotors();

  prevTime = micros();
}

// ─────────────────────────────────────────────────────────────────────────────
//  MAIN LOOP
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  // Constant high-frequency pulse execution poll
  motor1.runSpeed();
  motor2.runSpeed();

  // Timing gate (100Hz Control Loop)
  unsigned long now = micros();
  if (now - prevTime < LOOP_PERIOD_US) return;

  float dt = (float)(now - prevTime) * 1e-6f;   
  prevTime = now;

  if (dt > 0.05f) dt = 0.05f;
  if (dt < 0.001f) dt = 0.001f;

  SensorData s = readSensor();
  float angle = complementaryFilter(s.gyroX, s.accelY, s.accelZ, dt);
  float angleDev = angle - balanceSetpoint;   

  // Fall detection State Machine
  if (!isFallen) {
    if (fabsf(angleDev) > FALL_ANGLE) {
      isFallen = true;
      disableMotors();
      integral        = 0.0f;
      prevMeasurement = angle;
      prevDerivative  = 0.0f;
      lastAngle=angle;
      Serial.println(F("[FALLEN] Motors disabled."));
    }
  } else {
    if (fabsf(angleDev) < RECOVERY_ANGLE) {
      isFallen = false;
      filteredAngle   = angle;
      integral        = 0.0f;
      prevMeasurement = angle;
      prevDerivative  = 0.0f;
      lastAngle=angle;
      enableMotors();
      Serial.println(F("[RECOVERY] Motors re-enabled."));
    }
  }

  // Execute motor speed writes only if standing upright
  if (!isFallen) {
   float pidOutput = pidUpdate(balanceSetpoint, angle, dt);

float angleRate = (angle - lastAngle) / dt;
lastAngle = angle;

pidOutput -= velocityDamping * angleRate;

setMotorSpeed(pidOutput);
    

    motor1.runSpeed();  
    motor2.runSpeed();
  }

  // ── OPTIMIZATION: Serial prints are strictly isolated ──
  // CPU delays se bachne ke liye sirf troubleshooting ke vakt ise uncomment karein.
  /*
  static unsigned long diagTimer = 0;
  if (now - diagTimer >= 100000UL) {
    diagTimer = now;
    Serial.print(F("Angle: "));   Serial.print(angle,     2);
    Serial.print(F("  Int: "));   Serial.print(integral,  2);
    Serial.print(F("  State: ")); Serial.println(isFallen ? F("FALLEN") : F("BALANCING"));
  }
  */
}
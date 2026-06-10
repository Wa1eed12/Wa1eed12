// ============================================================
//   UNIVERSAL LINE FOLLOWER ROBOT — PID CONTROL
//   Author  : Waleed
//   Version : 4.3 — Anti-Jerk + Smart Recovery + Ultrasonic
// ============================================================
// ============================================================
//   ESP32-CAM SIGNAL RECEIVER
// ============================================================
#include <SoftwareSerial.h>
SoftwareSerial camSerial(10, 11); // RX=10, TX=11 (TX unused)
#define CMD_REVERSE_START 0xF1
bool reverseSignalReceived = false;

// ============================================================
//   [A] SURFACE TYPE
// ============================================================
#define BLACK_LINE_WHITE_SURFACE

// ============================================================
//   [B] MOTOR DIRECTION
// ============================================================
// #define LEFT_MOTOR_INVERTED
#define RIGHT_MOTOR_INVERTED

// ============================================================
//   [C] SENSOR PINS
//       A0 = physically RIGHT
//       A4 = physically LEFT
// ============================================================
#define SENSOR_COUNT 5
const int SENSOR_PINS[SENSOR_COUNT]   = {2, 3, 4, A3, A4};
const int SENSOR_WEIGHT[SENSOR_COUNT] = {2, 1, 0, -1, -2};

// ============================================================
//   [D] SENSOR THRESHOLD
// ============================================================
#define AUTO_CALIBRATE
#define FIXED_THRESHOLD 500

// ============================================================
//   [E] PID CONSTANTS — Anti-Jerk Tuned
// ============================================================
float KP = 18.0;
float KI = 0.0;
float KD = 65.0;

// ============================================================
//   [F] SPEED SETTINGS
// ============================================================
int BASE_SPEED = 60;
int MAX_SPEED  = 95;
int MIN_SPEED  = 35;
int TURN_SPEED = 50;
#define TURN_90_DURATION_MS 400

// ============================================================
//   [G] RECOVERY SETTINGS
// ============================================================
#define RECOVERY_TIMEOUT_MS  300
#define RECOVERY_SPEED       75

// ============================================================
//   [H] DEADBAND
// ============================================================
#define ERROR_DEADBAND  0.3

// ============================================================
//   [I] MOTOR DRIVER PINS — L298N
// ============================================================
#define LEFT_EN   5
#define LEFT_IN1  4
#define LEFT_IN2  7
#define RIGHT_EN  6
#define RIGHT_IN1 8
#define RIGHT_IN2 9

// ============================================================
//   [J] ULTRASONIC SENSOR PINS — HC-SR04
// ============================================================
#define TRIG_PIN         3
#define ECHO_PIN         2
#define US_TARGET_CM     20.0
#define US_KP            4.5
#define US_KD            2.0
#define US_ENGAGE_CM     30.0
#define US_CLEAR_CM      25.0
#define REALIGN_DURATION_MS 500

// ============================================================
//   [K] STARTUP DELAY
// ============================================================
#define STARTUP_DELAY_MS 3000

// ============================================================
//   [L] DEBUG MODE
// ============================================================
#define DEBUG_MODE

// ============================================================
//   END OF CONFIG
// ============================================================

// ------------------------------------------------------------
//   SURFACE LOGIC
// ------------------------------------------------------------
#ifdef BLACK_LINE_WHITE_SURFACE
  #define ON_LINE(raw)   ((raw) < sensorThreshold)
#else
  #define ON_LINE(raw)   ((raw) >= sensorThreshold)
#endif

// ------------------------------------------------------------
//   FORWARD DECLARATIONS
// ------------------------------------------------------------
void setMotors(int leftSpeed, int rightSpeed);
void stopMotors();
void checkUltrasonic();
float getDistanceCM();
void readSensors();
void runPID();
void calibrateSensors();
bool isAllWhite();
bool isAllBlack();
float calculateError();

// ------------------------------------------------------------
//   AVOIDANCE STATE MACHINE
// ------------------------------------------------------------
enum AvoidState { IDLE, AVOIDING, REALIGN };
AvoidState avoidState = IDLE;
unsigned long realignTimer = 0;
float usPrevError = 0.0;

// ------------------------------------------------------------
//   RECOVERY STATE
// ------------------------------------------------------------
bool  inRecovery        = false;
int   recoveryDirection = 1;
unsigned long recoveryStartTime = 0;

// ------------------------------------------------------------
//   GLOBAL STATE
// ------------------------------------------------------------
int   sensorThreshold = FIXED_THRESHOLD;
int   sensorRaw[SENSOR_COUNT];
bool  onLine[SENSOR_COUNT];

float lastError = 1.0;
float integral  = 0.0;

// ============================================================
//   SETUP
// ============================================================
void setup() {

  pinMode(LEFT_EN,   OUTPUT);
  pinMode(LEFT_IN1,  OUTPUT);
  pinMode(LEFT_IN2,  OUTPUT);
  pinMode(RIGHT_EN,  OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  stopMotors();

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  camSerial.begin(9600);

  Serial.begin(9600);
  Serial.println(F("=============================================="));
  Serial.println(F("  LINE FOLLOWER v4.3 — Waleed"));
  Serial.println(F("  Anti-Jerk + Smart Recovery + Ultrasonic"));
  Serial.println(F("=============================================="));
  Serial.print(F("KP="));  Serial.print(KP);
  Serial.print(F(" KI=")); Serial.print(KI);
  Serial.print(F(" KD=")); Serial.println(KD);
  Serial.println(F("CAM serial ready on pin 10"));

#ifdef AUTO_CALIBRATE
  calibrateSensors();
#else
  sensorThreshold = FIXED_THRESHOLD;
  Serial.print(F("Fixed threshold: "));
  Serial.println(sensorThreshold);
#endif

  Serial.print(F("Starting in "));
  Serial.print(STARTUP_DELAY_MS / 1000);
  Serial.println(F(" seconds..."));
  delay(STARTUP_DELAY_MS);
  Serial.println(F("GO"));
}

// ============================================================
//   MAIN LOOP
// ============================================================
void loop() {

  // ── CAM signal check (non-blocking, always first) ────────
  while (camSerial.available()) {
    byte b = camSerial.read();
    if (b == CMD_REVERSE_START) {
      reverseSignalReceived = true;
      Serial.println(F("[CAM] Reverse signal!"));
    }
  }

  // ── Act on reverse signal ─────────────────────────────────
  if (reverseSignalReceived) {
    reverseSignalReceived = false;
    inRecovery = false;
    integral   = 0.0;
    lastError  = 0.0;
    avoidState = IDLE;

    setMotors(-BASE_SPEED, -BASE_SPEED);
    delay(400);
    stopMotors();
    delay(100);
    return;
  }

  // ── Existing logic ────────────────────────────────────────
  checkUltrasonic();
  if (avoidState == IDLE) {
    readSensors();
    runPID();
  }
}

// ============================================================
//   CALIBRATION
// ============================================================
void calibrateSensors() {
  Serial.println(F("CALIBRATING — Keep robot on WHITE surface..."));

  long sumWhite[SENSOR_COUNT] = {0};
  int  samples = 200;

  for (int s = 0; s < samples; s++) {
    for (int i = 0; i < SENSOR_COUNT; i++) {
      sumWhite[i] += analogRead(SENSOR_PINS[i]);
    }
    delay(10);
  }

  long avgWhite = 0;
  Serial.print(F("White readings: "));
  for (int i = 0; i < SENSOR_COUNT; i++) {
    long w = sumWhite[i] / samples;
    avgWhite += w;
    Serial.print(w); Serial.print(F(" "));
  }
  Serial.println();
  avgWhite /= SENSOR_COUNT;

  long expectedBlack  = 100;
  sensorThreshold     = (int)((avgWhite + expectedBlack) / 2);
  sensorThreshold     = constrain(sensorThreshold, 150, 900);

  Serial.print(F("White avg:  ")); Serial.println(avgWhite);
  Serial.print(F("Threshold:  ")); Serial.println(sensorThreshold);
  Serial.println(F("Calibration done."));
}

// ============================================================
//   READ SENSORS
// ============================================================
void readSensors() {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    sensorRaw[i] = analogRead(SENSOR_PINS[i]);
    onLine[i]    = ON_LINE(sensorRaw[i]);
  }
}

// ============================================================
//   ALL WHITE
// ============================================================
bool isAllWhite() {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (onLine[i]) return false;
  }
  return true;
}

// ============================================================
//   ALL BLACK
// ============================================================
bool isAllBlack() {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (!onLine[i]) return false;
  }
  return true;
}

// ============================================================
//   CALCULATE WEIGHTED ERROR
// ============================================================
float calculateError() {
  int   active      = 0;
  float weightedSum = 0.0;

  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (onLine[i]) {
      weightedSum += SENSOR_WEIGHT[i];
      active++;
    }
  }

  if (active == 0) return lastError;
  return weightedSum / active;
}

// ============================================================
//   PID CONTROL
// ============================================================
void runPID() {

  bool white = isAllWhite();
  bool black = isAllBlack();

  // ---- CASE 1: Line lost — smart timed recovery ----
  if (white) {
    if (!inRecovery) {
      inRecovery        = true;
      recoveryStartTime = millis();
      recoveryDirection = (lastError >= 0) ? 1 : -1;

#ifdef DEBUG_MODE
      Serial.print(F("RECOVERY START — direction: "));
      Serial.println(recoveryDirection > 0 ? F("RIGHT") : F("LEFT"));
#endif
    }

    if (millis() - recoveryStartTime > RECOVERY_TIMEOUT_MS) {
      recoveryDirection = -recoveryDirection;
      recoveryStartTime = millis();

#ifdef DEBUG_MODE
      Serial.println(F("RECOVERY TIMEOUT — flipping direction"));
#endif
    }

    int spinL = RECOVERY_SPEED * recoveryDirection;
    int spinR = RECOVERY_SPEED * -recoveryDirection;
    setMotors(spinL, spinR);
    return;
  }

  // ---- Line found — exit recovery ----
  if (inRecovery) {
    inRecovery = false;
    integral   = 0.0;

#ifdef DEBUG_MODE
    Serial.println(F("LINE FOUND — resuming PID"));
#endif
  }

  // ---- CASE 2: Junction ----
  if (black) {
    lastError = 0.0;
    integral  = 0.0;
    setMotors(BASE_SPEED, BASE_SPEED);

#ifdef DEBUG_MODE
    Serial.println(F("JUNCTION — STRAIGHT"));
#endif
    return;
  }

  // ---- CASE 3: Normal PID line following ----
  float error = calculateError();

  if (error > -ERROR_DEADBAND && error < ERROR_DEADBAND) error = 0.0;

  integral        += error;
  integral         = constrain(integral, -50.0, 50.0);
  float derivative = error - lastError;
  float correction = (KP * error) + (KI * integral) + (KD * derivative);
  lastError        = error;

  int leftSpeed  = constrain((int)(BASE_SPEED + correction), MIN_SPEED, MAX_SPEED);
  int rightSpeed = constrain((int)(BASE_SPEED - correction), MIN_SPEED, MAX_SPEED);

  setMotors(leftSpeed, rightSpeed);

#ifdef DEBUG_MODE
  Serial.print(F("S:"));
  for (int i = 0; i < SENSOR_COUNT; i++) {
    Serial.print(onLine[i] ? 1 : 0);
  }
  Serial.print(F(" E:")); Serial.print(error, 2);
  Serial.print(F(" C:")); Serial.print(correction, 1);
  Serial.print(F(" L:")); Serial.print(leftSpeed);
  Serial.print(F(" R:")); Serial.println(rightSpeed);
#endif
}

// ============================================================
//   ULTRASONIC — Returns distance in cm
// ============================================================
float getDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return 999.0;
  return (duration * 0.0343) / 2.0;
}

// ============================================================
//   CHECK ULTRASONIC — PID-based avoidance state machine
// ============================================================
void checkUltrasonic() {

  float dist = getDistanceCM();

  // ---- STATE: IDLE ----
  if (avoidState == IDLE) {
    if (dist < US_ENGAGE_CM) {
      avoidState  = AVOIDING;
      usPrevError = 0.0;
      integral    = 0.0;

#ifdef DEBUG_MODE
      Serial.print(F("AVOID START — dist: "));
      Serial.println(dist);
#endif
    }
    return;
  }

  // ---- STATE: AVOIDING ----
  if (avoidState == AVOIDING) {

    float usError      = US_TARGET_CM - dist;
    float usDerivative = usError - usPrevError;
    float usCorrection = (US_KP * usError) + (US_KD * usDerivative);
    usPrevError        = usError;

    int leftSpeed  = constrain((int)(BASE_SPEED - usCorrection), MIN_SPEED, MAX_SPEED);
    int rightSpeed = constrain((int)(BASE_SPEED + usCorrection), MIN_SPEED, MAX_SPEED);

    setMotors(leftSpeed, rightSpeed);

#ifdef DEBUG_MODE
    Serial.print(F("AVOID — dist:"));   Serial.print(dist);
    Serial.print(F(" err:"));           Serial.print(usError);
    Serial.print(F(" cor:"));           Serial.print(usCorrection);
    Serial.print(F(" L:"));             Serial.print(leftSpeed);
    Serial.print(F(" R:"));             Serial.println(rightSpeed);
#endif

    if (dist > US_CLEAR_CM) {
      avoidState   = REALIGN;
      realignTimer = millis();
      integral     = 0.0;
      lastError    = 0.0;

#ifdef DEBUG_MODE
      Serial.println(F("OBSTACLE CLEAR — realigning to line"));
#endif
    }
    return;
  }

  // ---- STATE: REALIGN ----
  if (avoidState == REALIGN) {

    int leftSpeed  = constrain(BASE_SPEED + 20, MIN_SPEED, MAX_SPEED);
    int rightSpeed = constrain(BASE_SPEED - 20, MIN_SPEED, MAX_SPEED);
    setMotors(leftSpeed, rightSpeed);

    if (millis() - realignTimer > REALIGN_DURATION_MS) {
      avoidState = IDLE;
      integral   = 0.0;
      lastError  = 0.0;
      inRecovery = false;

#ifdef DEBUG_MODE
      Serial.println(F("REALIGN DONE — line PID resumed"));
#endif
    }
    return;
  }
}

// ============================================================
//   SET MOTORS
// ============================================================
void setMotors(int leftSpeed, int rightSpeed) {

  bool leftFwd = (leftSpeed >= 0);
#ifdef LEFT_MOTOR_INVERTED
  leftFwd = !leftFwd;
#endif
  digitalWrite(LEFT_IN1, leftFwd ? HIGH : LOW);
  digitalWrite(LEFT_IN2, leftFwd ? LOW  : HIGH);
  analogWrite(LEFT_EN, constrain(abs(leftSpeed), 0, 255));

  bool rightFwd = (rightSpeed >= 0);
#ifdef RIGHT_MOTOR_INVERTED
  rightFwd = !rightFwd;
#endif
  digitalWrite(RIGHT_IN1, rightFwd ? HIGH : LOW);
  digitalWrite(RIGHT_IN2, rightFwd ? LOW  : HIGH);
  analogWrite(RIGHT_EN, constrain(abs(rightSpeed), 0, 255));
}

// ============================================================
//   STOP MOTORS
// ============================================================
void stopMotors() {
  digitalWrite(LEFT_IN1,  LOW);
  digitalWrite(LEFT_IN2,  LOW);
  digitalWrite(RIGHT_IN1, LOW);
  digitalWrite(RIGHT_IN2, LOW);
  analogWrite(LEFT_EN,  0);
  analogWrite(RIGHT_EN, 0);
}
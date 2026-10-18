/*
 * Pan/Tilt Ultrasonic 3D Scanner
 * ------------------------------------------------------------
 * Mode 1: JOYSTICK  - joystick controls yaw/pitch smoothly (velocity control)
 * Mode 2: AUTO      - serpentine raster scan, 3 readings averaged per point
 *
 * Press the joystick button to toggle modes.
 *
 * Serial protocol (115200 baud, one message per line):
 *   D,<yaw>,<pitch>,<distance_cm>,<mode>   data point (mode: A = auto, J = joystick)
 *                                          distance = -1.0 means "no valid echo"
 *   S,START / S,END                        start / end of an automatic scan
 *   M,AUTO / M,JOY                         mode changed
 *   # text                                 info message (ignored by the mapper)
 */

#include <Servo.h>

// =====================================================================
//  PIN CONFIGURATION
// =====================================================================
const uint8_t JOY_X_PIN   = A0;
const uint8_t JOY_Y_PIN   = A1;
const uint8_t JOY_BTN_PIN = A2;

const uint8_t TRIG_PIN = 4;
const uint8_t ECHO_PIN = 3;

const uint8_t PITCH_SERVO_PIN = 6;   // servo1 - up/down
const uint8_t YAW_SERVO_PIN   = 5;   // servo2 - left/right

// =====================================================================
//  TUNABLE SETTINGS
// =====================================================================
// --- Serial ---
const long SERIAL_BAUD = 115200;

// --- Servo hardware ---
const int SERVO_MIN_US = 544;        // pulse width at 0 degrees
const int SERVO_MAX_US = 2400;       // pulse width at 180 degrees
const float YAW_LIMIT_MIN   = 0;     // absolute mechanical limits (degrees)
const float YAW_LIMIT_MAX   = 180;
const float PITCH_LIMIT_MIN = 0;
const float PITCH_LIMIT_MAX = 180;
const float YAW_HOME   = 90;         // "straight ahead"
const float PITCH_HOME = 90;         // "level"
const unsigned long SERVO_UPDATE_MS = 20;   // servo signal is 50 Hz

// --- Joystick ---
const int   JOY_DEADZONE   = 60;     // raw counts ignored around centre
const float JOY_MAX_SPEED  = 70.0;   // deg/s at full deflection
const float JOY_ACCEL      = 250.0;  // deg/s^2  (lower = smoother/slower response)
// Stick feel (push stick forward = tilt up vs down):
// Note: This only flips manual joystick stick direction on the hardware.
// The 3D plot orientation (up/down) is handled in mapper.py (press 'P' to flip).
const bool  JOY_INVERT_YAW   = true;
const bool  JOY_INVERT_PITCH = true;
const unsigned long JOY_SETTLE_MS = 150;    // stand still this long before measuring

// --- Automatic scan grid (degrees) ---
const int SCAN_YAW_MIN     = 20;
const int SCAN_YAW_MAX     = 160;
const int SCAN_YAW_STEP    = 5;      // ultrasonic beam is ~15 deg wide, 5-10 is sensible
const int SCAN_PITCH_MIN   = 60;
const int SCAN_PITCH_MAX   = 120;
const int SCAN_PITCH_STEP  = 10;

// --- Automatic scan speed ---
const float AUTO_AVG_SPEED   = 100.0;       // deg/s average speed between points (raise = faster)
const unsigned long AUTO_MIN_MOVE_MS = 40;  // never make a move quicker than this
const unsigned long AUTO_SETTLE_MS   = 60;  // let the servo/sensor stop vibrating

// --- Ultrasonic ---
const int   SAMPLES_PER_POINT  = 3;
const unsigned long ECHO_TIMEOUT_US = 25000UL;   // ~4.3 m
const float MIN_VALID_CM = 2.0;
const float MAX_VALID_CM = 400.0;
const unsigned long READING_GAP_MS = 15;         // gap between the 3 readings

// --- Button ---
const unsigned long DEBOUNCE_MS = 50;

// =====================================================================
//  GLOBAL STATE
// =====================================================================
enum Mode { MODE_JOYSTICK, MODE_AUTO };

Servo pitchServo;
Servo yawServo;

Mode  mode = MODE_JOYSTICK;
float yawAngle   = YAW_HOME;
float pitchAngle = PITCH_HOME;

// joystick state
int   joyCenterX = 512;
int   joyCenterY = 512;
float yawVelocity   = 0;
float pitchVelocity = 0;
unsigned long lastJoyUpdate = 0;
unsigned long lastMoveTime  = 0;
float lastReportedYaw   = -1000;
float lastReportedPitch = -1000;

// auto-scan state
int scanYawIndex       = 0;
int scanPitchIndex     = 0;
int scanDirection      = 1;     // +1 = yaw increasing, -1 = decreasing (serpentine)
int scanPitchDirection = 1;     // +1 = walking down rows, -1 = walking up rows

// =====================================================================
//  SETUP / LOOP
// =====================================================================
void setup() {
  Serial.begin(SERIAL_BAUD);
  setupPins();
  setupServos();
  calibrateJoystick();
  setMode(MODE_JOYSTICK);
  Serial.println(F("# Scanner ready"));
}

void loop() {
  handleModeButton();

  if (mode == MODE_JOYSTICK) {
    runJoystickStep();
  } else {
    runAutoScanStep();
  }
}

// =====================================================================
//  INITIALISATION
// =====================================================================
void setupPins() {
  pinMode(JOY_BTN_PIN, INPUT_PULLUP);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
}

void setupServos() {
  pitchServo.attach(PITCH_SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  yawServo.attach(YAW_SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  writeServos();
  delay(500);
}

// Read the stick at rest to find its true centre (don't touch it at power-up)
void calibrateJoystick() {
  long sumX = 0, sumY = 0;
  const int N = 32;
  for (int i = 0; i < N; i++) {
    sumX += analogRead(JOY_X_PIN);
    sumY += analogRead(JOY_Y_PIN);
    delay(5);
  }
  joyCenterX = sumX / N;
  joyCenterY = sumY / N;
}

// =====================================================================
//  MODE HANDLING
// =====================================================================
// Debounced button; toggles mode on each press
void handleModeButton() {
  static bool lastReading = HIGH;
  static bool stableState = HIGH;
  static unsigned long lastChange = 0;

  bool reading = digitalRead(JOY_BTN_PIN);
  if (reading != lastReading) {
    lastChange = millis();
    lastReading = reading;
  }
  if ((millis() - lastChange) > DEBOUNCE_MS && reading != stableState) {
    stableState = reading;
    if (stableState == LOW) {          // pressed
      setMode(mode == MODE_JOYSTICK ? MODE_AUTO : MODE_JOYSTICK);
    }
  }
}

void setMode(Mode newMode) {
  mode = newMode;
  if (mode == MODE_AUTO) {
    Serial.println(F("M,AUTO"));
    resetScan();
  } else {
    Serial.println(F("M,JOY"));
    yawVelocity = 0;
    pitchVelocity = 0;
    lastJoyUpdate = millis();
    lastMoveTime = millis();
    lastReportedYaw = -1000;
    lastReportedPitch = -1000;
  }
}

// =====================================================================
//  SERVO OUTPUT
// =====================================================================
int angleToMicros(float angle) {
  return SERVO_MIN_US + (int)((angle / 180.0f) * (SERVO_MAX_US - SERVO_MIN_US));
}

// Using microseconds gives much finer resolution than write(int degrees)
void writeServos() {
  yawServo.writeMicroseconds(angleToMicros(yawAngle));
  pitchServo.writeMicroseconds(angleToMicros(pitchAngle));
}

// =====================================================================
//  JOYSTICK MODE
// =====================================================================
// Returns -1.0 .. +1.0 with deadzone and a soft "expo" curve for fine control near centre
float readJoystickAxis(uint8_t pin, int center, bool invert) {
  int raw = analogRead(pin) - center;
  if (abs(raw) < JOY_DEADZONE) return 0;

  float n;
  if (raw > 0) n = (float)(raw - JOY_DEADZONE) / (float)(1023 - center - JOY_DEADZONE);
  else         n = (float)(raw + JOY_DEADZONE) / (float)(center - JOY_DEADZONE);

  n = constrain(n, -1.0f, 1.0f);
  n = 0.6f * n * n * n + 0.4f * n;
  return invert ? -n : n;
}

// Move 'current' toward 'target' by at most 'maxDelta'
float approach(float current, float target, float maxDelta) {
  float diff = target - current;
  if (diff >  maxDelta) return current + maxDelta;
  if (diff < -maxDelta) return current - maxDelta;
  return target;
}

void runJoystickStep() {
  unsigned long now = millis();
  if (now - lastJoyUpdate < SERVO_UPDATE_MS) return;
  float dt = (now - lastJoyUpdate) / 1000.0f;
  lastJoyUpdate = now;

  // Stick = desired SPEED (not position), ramped with limited acceleration
  float yawCmd   = readJoystickAxis(JOY_X_PIN, joyCenterX, JOY_INVERT_YAW)   * JOY_MAX_SPEED;
  float pitchCmd = readJoystickAxis(JOY_Y_PIN, joyCenterY, JOY_INVERT_PITCH) * JOY_MAX_SPEED;

  yawVelocity   = approach(yawVelocity,   yawCmd,   JOY_ACCEL * dt);
  pitchVelocity = approach(pitchVelocity, pitchCmd, JOY_ACCEL * dt);

  yawAngle   += yawVelocity * dt;
  pitchAngle += pitchVelocity * dt;

  if (yawAngle < YAW_LIMIT_MIN)     { yawAngle = YAW_LIMIT_MIN;     yawVelocity = 0; }
  if (yawAngle > YAW_LIMIT_MAX)     { yawAngle = YAW_LIMIT_MAX;     yawVelocity = 0; }
  if (pitchAngle < PITCH_LIMIT_MIN) { pitchAngle = PITCH_LIMIT_MIN; pitchVelocity = 0; }
  if (pitchAngle > PITCH_LIMIT_MAX) { pitchAngle = PITCH_LIMIT_MAX; pitchVelocity = 0; }

  writeServos();

  bool moving = (fabs(yawVelocity) > 0.3f) || (fabs(pitchVelocity) > 0.3f) ||
                (yawCmd != 0) || (pitchCmd != 0);
  if (moving) {
    lastMoveTime = now;
  } else {
    reportJoystickPointIfNeeded(now);
  }
}

// Once the sensor has been still for a moment, take a 3-sample point (if we moved since last one)
void reportJoystickPointIfNeeded(unsigned long now) {
  if (now - lastMoveTime < JOY_SETTLE_MS) return;

  bool changed = fabs(yawAngle - lastReportedYaw) >= 0.5f ||
                 fabs(pitchAngle - lastReportedPitch) >= 0.5f;
  if (!changed) return;

  float d = measureDistance();
  sendPoint(yawAngle, pitchAngle, d, 'J');
  lastReportedYaw = yawAngle;
  lastReportedPitch = pitchAngle;
  lastJoyUpdate = millis();   // measurement took time; avoid a big dt next step
}

// =====================================================================
//  AUTOMATIC SCAN MODE
// =====================================================================
int yawStepsCount()   { return (SCAN_YAW_MAX - SCAN_YAW_MIN) / SCAN_YAW_STEP + 1; }
int pitchStepsCount() { return (SCAN_PITCH_MAX - SCAN_PITCH_MIN) / SCAN_PITCH_STEP + 1; }

void resetScan() {
  scanYawIndex = 0;
  scanPitchIndex = 0;
  scanDirection = 1;
  scanPitchDirection = 1;
  Serial.println(F("S,START"));
}

// One scan point: move -> settle -> measure -> send -> advance
void runAutoScanStep() {
  float targetYaw   = SCAN_YAW_MIN   + scanYawIndex   * SCAN_YAW_STEP;
  float targetPitch = SCAN_PITCH_MIN + scanPitchIndex * SCAN_PITCH_STEP;

  if (!smoothMoveTo(targetYaw, targetPitch)) return;   // aborted by button
  if (!waitMs(AUTO_SETTLE_MS)) return;

  float d = measureDistance();
  sendPoint(targetYaw, targetPitch, d, 'A');
  advanceScanIndex();
}

// Serpentine order: left->right on one row, right->left on the next (minimal travel).
// Bidirectional pitch: walks down row-by-row, then walks back up without jumping.
void advanceScanIndex() {
  scanYawIndex += scanDirection;

  if (scanYawIndex < 0 || scanYawIndex >= yawStepsCount()) {
    scanYawIndex -= scanDirection;
    scanDirection = -scanDirection;

    if (pitchStepsCount() <= 1) {
      Serial.println(F("S,END"));
      Serial.println(F("S,START"));
      return;
    }

    scanPitchIndex += scanPitchDirection;

    if (scanPitchIndex >= pitchStepsCount() || scanPitchIndex < 0) {
      Serial.println(F("S,END"));
      scanPitchDirection = -scanPitchDirection;
      scanPitchIndex += 2 * scanPitchDirection;
      Serial.println(F("S,START"));
    }
  }
}

// Both axes move together on a smoothstep (ease-in/ease-out) curve:
// no sudden start or stop, and duration scales with distance so short hops are quick.
// Returns false if the mode was switched during the move.
bool smoothMoveTo(float targetYaw, float targetPitch) {
  float startYaw = yawAngle;
  float startPitch = pitchAngle;
  float distance = max(fabs(targetYaw - startYaw), fabs(targetPitch - startPitch));
  if (distance < 0.01f) return true;

  unsigned long duration = (unsigned long)(distance / AUTO_AVG_SPEED * 1000.0f);
  if (duration < AUTO_MIN_MOVE_MS) duration = AUTO_MIN_MOVE_MS;

  unsigned long startTime = millis();
  while (true) {
    handleModeButton();
    if (mode != MODE_AUTO) return false;

    float t = (float)(millis() - startTime) / (float)duration;
    if (t > 1.0f) t = 1.0f;
    float s = t * t * (3.0f - 2.0f * t);

    yawAngle   = startYaw   + (targetYaw   - startYaw)   * s;
    pitchAngle = startPitch + (targetPitch - startPitch) * s;
    writeServos();

    if (t >= 1.0f) break;
    delay(SERVO_UPDATE_MS);
  }
  return true;
}

// Delay that still listens to the mode button. Returns false if mode changed.
bool waitMs(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    handleModeButton();
    if (mode != MODE_AUTO) return false;
  }
  return true;
}

// =====================================================================
//  ULTRASONIC SENSOR
// =====================================================================
// One reading in cm, or -1 if no valid echo
float readDistanceOnce() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  if (duration == 0) return -1;

  float cm = duration * 0.0343f / 2.0f;
  if (cm < MIN_VALID_CM || cm > MAX_VALID_CM) return -1;
  return cm;
}

// Average of SAMPLES_PER_POINT readings (invalid ones are skipped); -1 if all invalid
float measureDistance() {
  float sum = 0;
  int valid = 0;
  for (int i = 0; i < SAMPLES_PER_POINT; i++) {
    float d = readDistanceOnce();
    if (d > 0) { sum += d; valid++; }
    delay(READING_GAP_MS);
  }
  return valid > 0 ? sum / valid : -1.0f;
}

// =====================================================================
//  SERIAL OUTPUT
// =====================================================================
void sendPoint(float yaw, float pitch, float distance, char modeChar) {
  Serial.print(F("D,"));
  Serial.print(yaw, 1);
  Serial.print(',');
  Serial.print(pitch, 1);
  Serial.print(',');
  Serial.print(distance, 1);
  Serial.print(',');
  Serial.println(modeChar);
}

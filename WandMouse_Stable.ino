#include <Wire.h>
#include <BleMouse.h>
#include <Preferences.h>
#include <math.h>

BleMouse bleMouse("ESP32 Air Mouse", "ESP32-Lab", 100);
Preferences prefs;

// ===================== MPU6500 =====================
uint8_t mpuAddr = 0x68;
#define WHO_AM_I 0x75
#define PWR_MGMT_1 0x6B
#define CONFIG_REG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B

struct ImuEvent {
  float ax, ay, az;   // m/s^2
  float gx, gy, gz;   // rad/s
};

struct MotionState {
  float cursorRateX = 0.0f;
  float cursorRateY = 0.0f;
  float scrollInput = 0.0f;
  float dropInput = 0.0f;
  bool idleLocked = false;
};

// Axis ids: 0=X, 1=Y, 2=Z. Flip signs first if direction is reversed.
const uint8_t CURSOR_X_GYRO_AXIS = 2;
const uint8_t CURSOR_Y_GYRO_AXIS = 0;
const float CURSOR_X_SIGN = -1.0f;
const float CURSOR_Y_SIGN =  1.0f;

const uint8_t SCROLL_ACCEL_AXIS = 1;
const float SCROLL_SIGN = 1.0f;
const float SCROLL_GYRO_SCALE = 8.0f;     // Uses vertical wand motion, so both scroll directions work.

const uint8_t DROP_ACCEL_AXIS = 1;
const float DROP_SIGN = -1.0f;

// +/-8g accel = 4096 LSB/g
const float ACCEL_SCALE = 9.80665f / 4096.0f;

// +/-500 deg/s gyro = 65.5 LSB/(deg/s)
const float GYRO_SCALE = (PI / 180.0f) / 65.5f;

// ===================== Filtering =====================
const float KALMAN_Q = 0.012f;
const float KALMAN_R = 0.24f;
const float KALMAN_INITIAL_P = 1.0f;

struct Kalman1D {
  float x = 0.0f;
  float p = KALMAN_INITIAL_P;
  float k = 0.0f;
  bool initialized = false;

  void reset(float initialValue = 0.0f) {
    x = initialValue;
    p = KALMAN_INITIAL_P;
    k = 0.0f;
    initialized = true;
  }

  float update(float measurement) {
    if (!initialized) {
      reset(measurement);
      return x;
    }

    p += KALMAN_Q;
    k = p / (p + KALMAN_R);
    x += k * (measurement - x);
    p *= (1.0f - k);
    return x;
  }
};

Kalman1D cursorXFilter;
Kalman1D cursorYFilter;
Kalman1D aimXFilter;
Kalman1D aimYFilter;
Kalman1D scrollFilter;
Kalman1D dropFilter;

// ===================== Cursor Tuning =====================
float GYRO_SENSITIVITY = 18.0f;          // Lower = calmer cursor.
const float DEADZONE = 0.045f;           // Higher = less idle drift.
const float SMOOTH_ALPHA = 0.22f;
const float OUTPUT_MIN_STEP = 1.25f;     // Prevents tiny 1-pixel idle creep.
const unsigned long LOOP_INTERVAL_MS = 10;

const float STILL_GYRO_THRESHOLD = 0.070f;
const unsigned long STILL_LOCK_MS = 320;
const float AUTO_ZERO_ALPHA = 0.0035f;

bool DEBUG_MODE = false;
bool mpuPresent = false;

// ===================== IR Click / Drag =====================
#define IR_CLICK_PIN 27
const bool IR_ACTIVE_LOW = true;

const unsigned long IR_DEBOUNCE_MS = 75;
const unsigned long IR_CLICK_COOLDOWN_MS = 240;
const unsigned long CLICK_MIN_PRESS_MS = 45;
const unsigned long CLICK_MAX_PRESS_MS = 900;
const unsigned long HOLD_DRAG_MS = 1100;
const unsigned long CALIBRATION_HOLD_MS = 2600;

const float CLICK_CANCEL_GYRO = 0.55f;

enum IrMode {
  IR_IDLE,
  IR_PENDING,
  IR_SCROLLING,
  IR_DRAGGING,
  IR_DROPPED,
  IR_CALIBRATION_CAPTURE
};

IrMode irMode = IR_IDLE;
bool lastRawIrTouched = false;
bool stableIrTouched = false;
bool previousStableIrTouched = false;
bool suppressClickUntilRelease = false;
bool irMotionExceededForCalibration = false;

unsigned long lastIrChangeTime = 0;
unsigned long lastIrClickTime = 0;
unsigned long irPressStartTime = 0;
unsigned long dragStartTime = 0;
float irMaxGyroMotion = 0.0f;
float irMaxAccelMotion = 0.0f;

bool leftButtonDown = false;
bool connectedLastLoop = false;

// ===================== Scroll =====================
const float SCROLL_DEADZONE = 0.48f;
const float SCROLL_SENSITIVITY = 0.42f;
const int8_t SCROLL_MAX_STEP = 5;
const unsigned long SCROLL_COOLDOWN_MS = 35;
const unsigned long SCROLL_HOLD_ARM_MS = 150;
const unsigned long SCROLL_AFTER_ACTION_LOCKOUT_MS = 360;

// ===================== Drop Gesture =====================
const float DROP_GESTURE_THRESHOLD = 7.4f;
const unsigned long DROP_GESTURE_COOLDOWN_MS = 850;
const unsigned long DROP_MIN_DRAG_MS = 220;

// ===================== IMU Calibration =====================
const int IMU_CALIBRATION_SAMPLES = 300;
const unsigned long IMU_CALIBRATION_SAMPLE_INTERVAL_MS = 5;
const float CALIBRATION_HOLD_GYRO_LIMIT = 0.18f;
const float CALIBRATION_HOLD_ACCEL_LIMIT = 1.2f;

float gyroOffsetX = 0.0f;
float gyroOffsetY = 0.0f;
float gyroOffsetZ = 0.0f;

float accelBaselineX = 0.0f;
float accelBaselineY = 0.0f;
float accelBaselineZ = 0.0f;

bool imuCalibrated = false;
bool imuCalibrationActive = false;
int imuCalibrationCount = 0;
double imuSumAX = 0.0, imuSumAY = 0.0, imuSumAZ = 0.0;
double imuSumGX = 0.0, imuSumGY = 0.0, imuSumGZ = 0.0;
unsigned long lastImuCalibrationSample = 0;

// ===================== Screen Calibration =====================
uint16_t screenWidth = 1920;
uint16_t screenHeight = 1080;

const bool ENABLE_SCREEN_CALIBRATION = true;
const float CALIBRATED_RESPONSE = 0.32f;
const int8_t MAX_CALIBRATED_STEP = 38;
const float CALIBRATION_MIN_RANGE = 0.05f;

bool screenCalibrated = false;
bool calibrationMode = false;
uint8_t calibrationStep = 0;

float aimX = 0.0f;
float aimY = 0.0f;
float filteredAimX = 0.0f;
float filteredAimY = 0.0f;
unsigned long lastAimUpdateTime = 0;

float cornerAimX[4] = {0, 0, 0, 0};
float cornerAimY[4] = {0, 0, 0, 0};
float calLeftX = 0.0f;
float calRightX = 1.0f;
float calTopY = 0.0f;
float calBottomY = 1.0f;

float virtualCursorX = screenWidth / 2.0f;
float virtualCursorY = screenHeight / 2.0f;

// ===================== Runtime State =====================
MotionState motion;
float smoothX = 0.0f;
float smoothY = 0.0f;
float scrollAccumulator = 0.0f;

unsigned long lastLoopTime = 0;
unsigned long lastStillStartTime = 0;
unsigned long lastScrollTime = 0;
unsigned long lastClickOrDropTime = 0;
unsigned long lastDropGestureTime = 0;

bool serialScrollMode = false;
String serialBuffer;

// ===================== Prototypes =====================
void writeReg(uint8_t reg, uint8_t val);
uint8_t readReg(uint8_t reg);
void scanI2CBus();
bool mpu6500Begin();
bool mpu6500BeginAt(uint8_t address);
void readMPU6500(ImuEvent &imu);

void handleSerialCommands();
void processSerialCommand(String cmd);
void printCommandHelp();

void loadScreenCalibration();
void saveScreenCalibration();
void clearScreenCalibration();
void startScreenCalibration();
void captureCalibrationCorner();
void finishScreenCalibration();
void printCalibrationPrompt();
void setScreenSize(uint16_t width, uint16_t height);
void centerAimTracking(bool printMessage);

void loadImuCalibration();
void saveImuCalibration();
void clearImuCalibration();
void startImuCalibration();
void updateImuCalibration(unsigned long now);
void printCalibrationStatus();

void updateMotionState(ImuEvent &imu, unsigned long now);
void handleCursorMovement();
void handleCalibratedCursorMovement();
void handleIRInput();
bool handleScrolling();
void handleDropGesture();
void updateIrMotionGuard(ImuEvent &imu);

void pressLeftButton();
void releaseLeftButton();
void releaseAllMouseButtons();
void resetMotionFilters();

float gyroAxis(ImuEvent &imu, uint8_t axis);
float accelAxis(ImuEvent &imu, uint8_t axis);
float correctedAccelX(ImuEvent &imu);
float correctedAccelY(ImuEvent &imu);
float correctedAccelZ(ImuEvent &imu);
float correctedAccelAxis(ImuEvent &imu, uint8_t axis);
float mapFloat(float value, float inMin, float inMax, float outMin, float outMax);
float clampFloat(float value, float low, float high);
float applyDeadzone(float value, float threshold);
float applyEMA(float previous, float current, float alpha);
const char *irModeName();
void printDebug(float dx, float dy);

// ===================== Setup / Loop =====================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println(F("ESP32 BLE Wand Mouse - stable click/drag build"));

  prefs.begin("airmouse", false);
  loadScreenCalibration();
  loadImuCalibration();
  centerAimTracking(false);

  pinMode(IR_CLICK_PIN, IR_ACTIVE_LOW ? INPUT_PULLUP : INPUT);
  Serial.printf("[IR] Sensor on GPIO %d, active %s\n",
                IR_CLICK_PIN, IR_ACTIVE_LOW ? "LOW" : "HIGH");

  Wire.begin(21, 22);
  Wire.setClock(100000);

  scanI2CBus();

  mpuPresent = mpu6500Begin();
  if (!mpuPresent) {
    Serial.println(F("[ERROR] MPU6500 not found on 0x68 or 0x69."));
    Serial.println(F("[ERROR] Mouse movement disabled; IR click test still works."));
  } else {
    Serial.printf("[OK] MPU6500 detected at I2C address 0x%02X\n", mpuAddr);

    if (!imuCalibrated) {
      startImuCalibration();
    } else {
      Serial.println(F("[IMU] Saved gyro + accel calibration loaded."));
    }
  }

  Serial.println(F("[BLE] Starting BLE Mouse..."));
  bleMouse.begin();
  Serial.println(F("[BLE] Advertising. Pair with 'ESP32 Air Mouse'."));
  printCommandHelp();
}

void loop() {
  unsigned long now = millis();

  if (now - lastLoopTime < LOOP_INTERVAL_MS) return;
  lastLoopTime = now;

  handleSerialCommands();

  if (mpuPresent && imuCalibrationActive) {
    updateImuCalibration(now);
    return;
  }

  if (!bleMouse.isConnected()) {
    connectedLastLoop = false;
    leftButtonDown = false;

    static unsigned long lastDot = 0;
    if (now - lastDot > 2000) {
      Serial.print('.');
      lastDot = now;
    }
    return;
  }

  if (!connectedLastLoop) {
    releaseAllMouseButtons();
    connectedLastLoop = true;
    lastClickOrDropTime = now;
    Serial.println(F("[BLE] Connected. Buttons released."));
  }

  handleIRInput();

  if (!mpuPresent) return;

  ImuEvent imu;
  readMPU6500(imu);

  updateIrMotionGuard(imu);
  updateMotionState(imu, now);
  handleDropGesture();

  if (calibrationMode) {
    handleCursorMovement();
    return;
  }

  if (handleScrolling()) return;

  if (ENABLE_SCREEN_CALIBRATION && screenCalibrated) {
    handleCalibratedCursorMovement();
  } else {
    handleCursorMovement();
  }
}

// ===================== Serial Commands =====================
void handleSerialCommands() {
  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == '\r') continue;

    if (c == '\n') {
      serialBuffer.trim();
      if (serialBuffer.length() > 0) {
        processSerialCommand(serialBuffer);
      }
      serialBuffer = "";
      return;
    }

    if (c >= 32 && c <= 126 && serialBuffer.length() < 96) {
      serialBuffer += c;
    }
  }
}

void processSerialCommand(String cmd) {
  cmd.trim();

  String lower = cmd;
  lower.toLowerCase();

  if (lower == "help" || lower == "?") {
    printCommandHelp();
    return;
  }

  if (lower == "debug" || lower == "d") {
    DEBUG_MODE = !DEBUG_MODE;
    Serial.printf("[DEBUG] %s\n", DEBUG_MODE ? "ON" : "OFF");
    return;
  }

  if (lower == "imu_cal") {
    startImuCalibration();
    return;
  }

  if (lower == "imu_reset") {
    clearImuCalibration();
    return;
  }

  if (lower == "cal_status") {
    printCalibrationStatus();
    return;
  }

  if (lower == "click" || lower == "left" || lower == "leftclick") {
    if (bleMouse.isConnected()) {
      releaseAllMouseButtons();
      bleMouse.click(MOUSE_LEFT);
      lastIrClickTime = millis();
      lastClickOrDropTime = millis();
      Serial.println(F("[CLICK] LEFT"));
    } else {
      Serial.println(F("[CLICK] BLE mouse is not connected."));
    }
    return;
  }

  if (lower == "cal" || lower == "c" || lower == "calibrate") {
    startScreenCalibration();
    return;
  }

  if (lower == "resetcal" || lower == "r") {
    clearScreenCalibration();
    return;
  }

  if (lower == "center" || lower == "sync") {
    centerAimTracking(true);
    return;
  }

  if (lower == "scroll" || lower == "scroll toggle") {
    serialScrollMode = !serialScrollMode;
    scrollAccumulator = 0.0f;
    Serial.printf("[SCROLL] Serial scroll mode %s\n", serialScrollMode ? "ON" : "OFF");
    return;
  }

  if (lower == "scroll on") {
    serialScrollMode = true;
    scrollAccumulator = 0.0f;
    Serial.println(F("[SCROLL] Serial scroll mode ON"));
    return;
  }

  if (lower == "scroll off") {
    serialScrollMode = false;
    scrollAccumulator = 0.0f;
    Serial.println(F("[SCROLL] Serial scroll mode OFF"));
    return;
  }

  if (lower.startsWith("size ")) {
    int width = 0;
    int height = 0;

    if (sscanf(lower.c_str(), "size %d %d", &width, &height) == 2 &&
        width >= 200 && height >= 200 && width <= 10000 && height <= 10000) {
      setScreenSize((uint16_t)width, (uint16_t)height);
    } else {
      Serial.println(F("[SIZE] Use format: size 1920 1080"));
    }
    return;
  }

  Serial.println(F("[CMD] Unknown command. Type 'help'."));
}

void printCommandHelp() {
  Serial.println();
  Serial.println(F("[HELP] Commands:"));
  Serial.println(F("  help              - show this list"));
  Serial.println(F("  debug             - toggle debug output"));
  Serial.println(F("  imu_cal           - recalibrate gyro + accelerometer baseline"));
  Serial.println(F("  imu_reset         - clear saved IMU calibration"));
  Serial.println(F("  cal               - start 4-corner screen calibration"));
  Serial.println(F("  resetcal          - clear screen calibration"));
  Serial.println(F("  cal_status        - print screen + IMU calibration status"));
  Serial.println(F("  click             - send one left click for testing"));
  Serial.println(F("  center            - reset aim/virtual cursor to center"));
  Serial.println(F("  size 1920 1080    - set screen size"));
  Serial.println(F("  scroll on|off     - force MPU scroll mode without holding IR"));
  Serial.println();
}

// ===================== Screen Calibration =====================
void loadScreenCalibration() {
  screenWidth = prefs.getUShort("sw", screenWidth);
  screenHeight = prefs.getUShort("sh", screenHeight);

  screenCalibrated = prefs.getBool("cal", false);
  if (screenCalibrated) {
    calLeftX = prefs.getFloat("leftX", calLeftX);
    calRightX = prefs.getFloat("rightX", calRightX);
    calTopY = prefs.getFloat("topY", calTopY);
    calBottomY = prefs.getFloat("botY", calBottomY);
  }

  virtualCursorX = screenWidth / 2.0f;
  virtualCursorY = screenHeight / 2.0f;

  Serial.printf("[SIZE] Screen size: %u x %u\n", screenWidth, screenHeight);
  Serial.printf("[CAL] Screen calibration: %s\n", screenCalibrated ? "LOADED" : "not set");
  Serial.printf("[CAL] Hold IR still for %lu ms or type 'cal' to calibrate corners.\n",
                CALIBRATION_HOLD_MS);
}

void saveScreenCalibration() {
  prefs.putBool("cal", screenCalibrated);
  prefs.putFloat("leftX", calLeftX);
  prefs.putFloat("rightX", calRightX);
  prefs.putFloat("topY", calTopY);
  prefs.putFloat("botY", calBottomY);
  prefs.putUShort("sw", screenWidth);
  prefs.putUShort("sh", screenHeight);
}

void clearScreenCalibration() {
  screenCalibrated = false;
  calibrationMode = false;
  calibrationStep = 0;
  prefs.putBool("cal", false);
  centerAimTracking(false);
  Serial.println(F("[CAL] Screen calibration cleared. Relative air-mouse mode active."));
}

void setScreenSize(uint16_t width, uint16_t height) {
  screenWidth = width;
  screenHeight = height;
  virtualCursorX = clampFloat(virtualCursorX, 0.0f, screenWidth - 1.0f);
  virtualCursorY = clampFloat(virtualCursorY, 0.0f, screenHeight - 1.0f);

  prefs.putUShort("sw", screenWidth);
  prefs.putUShort("sh", screenHeight);

  Serial.printf("[SIZE] Screen size set to %u x %u\n", screenWidth, screenHeight);
}

void startScreenCalibration() {
  if (!ENABLE_SCREEN_CALIBRATION) {
    Serial.println(F("[CAL] ENABLE_SCREEN_CALIBRATION is false."));
    return;
  }

  if (!mpuPresent) {
    Serial.println(F("[CAL] Cannot calibrate: MPU6500 is not detected."));
    return;
  }

  if (imuCalibrationActive) {
    Serial.println(F("[CAL] Wait for IMU calibration to finish first."));
    return;
  }

  releaseAllMouseButtons();

  calibrationMode = true;
  screenCalibrated = false;
  calibrationStep = 0;

  aimX = 0.0f;
  aimY = 0.0f;
  filteredAimX = 0.0f;
  filteredAimY = 0.0f;
  smoothX = 0.0f;
  smoothY = 0.0f;
  scrollAccumulator = 0.0f;
  lastAimUpdateTime = millis();
  resetMotionFilters();

  if (stableIrTouched) suppressClickUntilRelease = true;

  Serial.println();
  Serial.println(F("[CAL] 4-corner screen calibration started."));
  Serial.println(F("[CAL] Move the pointer to the requested corner, then tap IR once."));
  Serial.println(F("[CAL] IR taps record corners instead of clicking."));
  printCalibrationPrompt();
}

void printCalibrationPrompt() {
  const char *names[4] = {"TOP-LEFT", "TOP-RIGHT", "BOTTOM-RIGHT", "BOTTOM-LEFT"};
  if (calibrationStep < 4) {
    Serial.printf("[CAL] Step %u/4: move pointer to %s, then tap IR.\n",
                  calibrationStep + 1, names[calibrationStep]);
  }
}

void captureCalibrationCorner() {
  if (!calibrationMode || calibrationStep >= 4) return;

  cornerAimX[calibrationStep] = filteredAimX;
  cornerAimY[calibrationStep] = filteredAimY;

  const char *names[4] = {"TOP-LEFT", "TOP-RIGHT", "BOTTOM-RIGHT", "BOTTOM-LEFT"};
  Serial.printf("[CAL] Captured %s: aimX=%.4f aimY=%.4f\n",
                names[calibrationStep], filteredAimX, filteredAimY);

  calibrationStep++;
  suppressClickUntilRelease = true;
  irMode = IR_CALIBRATION_CAPTURE;

  if (calibrationStep >= 4) {
    finishScreenCalibration();
  } else {
    printCalibrationPrompt();
  }
}

void finishScreenCalibration() {
  calLeftX = (cornerAimX[0] + cornerAimX[3]) * 0.5f;
  calRightX = (cornerAimX[1] + cornerAimX[2]) * 0.5f;
  calTopY = (cornerAimY[0] + cornerAimY[1]) * 0.5f;
  calBottomY = (cornerAimY[2] + cornerAimY[3]) * 0.5f;

  float rangeX = fabsf(calRightX - calLeftX);
  float rangeY = fabsf(calBottomY - calTopY);

  if (rangeX < CALIBRATION_MIN_RANGE || rangeY < CALIBRATION_MIN_RANGE) {
    calibrationMode = false;
    screenCalibrated = false;
    Serial.println(F("[CAL] Failed: range too small. Move farther between corners and retry."));
    return;
  }

  screenCalibrated = true;
  calibrationMode = false;
  saveScreenCalibration();

  virtualCursorX = mapFloat(filteredAimX, calLeftX, calRightX, 0.0f, screenWidth - 1.0f);
  virtualCursorY = mapFloat(filteredAimY, calTopY, calBottomY, 0.0f, screenHeight - 1.0f);
  virtualCursorX = clampFloat(virtualCursorX, 0.0f, screenWidth - 1.0f);
  virtualCursorY = clampFloat(virtualCursorY, 0.0f, screenHeight - 1.0f);

  Serial.println(F("[CAL] Complete. Calibrated screen mode is active."));
  Serial.printf("[CAL] X left/right: %.4f / %.4f | Y top/bottom: %.4f / %.4f\n",
                calLeftX, calRightX, calTopY, calBottomY);
}

void centerAimTracking(bool printMessage) {
  releaseAllMouseButtons();

  virtualCursorX = screenWidth / 2.0f;
  virtualCursorY = screenHeight / 2.0f;

  if (screenCalibrated) {
    aimX = (calLeftX + calRightX) * 0.5f;
    aimY = (calTopY + calBottomY) * 0.5f;
  } else {
    aimX = 0.0f;
    aimY = 0.0f;
  }

  filteredAimX = aimX;
  filteredAimY = aimY;
  smoothX = 0.0f;
  smoothY = 0.0f;
  scrollAccumulator = 0.0f;
  lastAimUpdateTime = millis();
  resetMotionFilters();

  if (printMessage) {
    Serial.println(F("[CENTER] Aim and virtual cursor reset to center."));
  }
}

// ===================== IMU Calibration =====================
void loadImuCalibration() {
  imuCalibrated = prefs.getBool("imuCal", false);

  if (imuCalibrated) {
    gyroOffsetX = prefs.getFloat("gox", 0.0f);
    gyroOffsetY = prefs.getFloat("goy", 0.0f);
    gyroOffsetZ = prefs.getFloat("goz", 0.0f);
    accelBaselineX = prefs.getFloat("abx", 0.0f);
    accelBaselineY = prefs.getFloat("aby", 0.0f);
    accelBaselineZ = prefs.getFloat("abz", 0.0f);
  }
}

void saveImuCalibration() {
  prefs.putBool("imuCal", true);
  prefs.putFloat("gox", gyroOffsetX);
  prefs.putFloat("goy", gyroOffsetY);
  prefs.putFloat("goz", gyroOffsetZ);
  prefs.putFloat("abx", accelBaselineX);
  prefs.putFloat("aby", accelBaselineY);
  prefs.putFloat("abz", accelBaselineZ);
}

void clearImuCalibration() {
  imuCalibrationActive = false;
  imuCalibrated = false;

  gyroOffsetX = 0.0f;
  gyroOffsetY = 0.0f;
  gyroOffsetZ = 0.0f;
  accelBaselineX = 0.0f;
  accelBaselineY = 0.0f;
  accelBaselineZ = 0.0f;

  prefs.putBool("imuCal", false);
  prefs.remove("gox");
  prefs.remove("goy");
  prefs.remove("goz");
  prefs.remove("abx");
  prefs.remove("aby");
  prefs.remove("abz");

  resetMotionFilters();
  Serial.println(F("[IMU] Saved IMU calibration cleared. Run 'imu_cal' while wand is still."));
}

void startImuCalibration() {
  if (!mpuPresent) {
    Serial.println(F("[IMU] Cannot calibrate: MPU6500 is not detected."));
    return;
  }

  if (calibrationMode) {
    Serial.println(F("[IMU] Finish screen calibration first."));
    return;
  }

  releaseAllMouseButtons();

  imuCalibrationActive = true;
  imuCalibrationCount = 0;
  imuSumAX = imuSumAY = imuSumAZ = 0.0;
  imuSumGX = imuSumGY = imuSumGZ = 0.0;
  lastImuCalibrationSample = 0;

  Serial.println();
  Serial.println(F("[IMU] Gyro + accelerometer baseline calibration started."));
  Serial.println(F("[IMU] Hold the wand still in your normal pointing position."));
  Serial.println(F("[IMU] Keep the MPU tip steady. Do not touch the IR sensor."));
  Serial.printf("[IMU] Collecting %d samples...\n", IMU_CALIBRATION_SAMPLES);
}

void updateImuCalibration(unsigned long now) {
  if (!imuCalibrationActive) return;

  if (lastImuCalibrationSample != 0 &&
      now - lastImuCalibrationSample < IMU_CALIBRATION_SAMPLE_INTERVAL_MS) {
    return;
  }
  lastImuCalibrationSample = now;

  ImuEvent imu;
  readMPU6500(imu);

  imuSumAX += imu.ax;
  imuSumAY += imu.ay;
  imuSumAZ += imu.az;
  imuSumGX += imu.gx;
  imuSumGY += imu.gy;
  imuSumGZ += imu.gz;
  imuCalibrationCount++;

  if (imuCalibrationCount % 75 == 0) {
    Serial.printf("[IMU] Samples: %d/%d\n", imuCalibrationCount, IMU_CALIBRATION_SAMPLES);
  }

  if (imuCalibrationCount < IMU_CALIBRATION_SAMPLES) return;

  gyroOffsetX = imuSumGX / imuCalibrationCount;
  gyroOffsetY = imuSumGY / imuCalibrationCount;
  gyroOffsetZ = imuSumGZ / imuCalibrationCount;

  accelBaselineX = imuSumAX / imuCalibrationCount;
  accelBaselineY = imuSumAY / imuCalibrationCount;
  accelBaselineZ = imuSumAZ / imuCalibrationCount;

  imuCalibrated = true;
  imuCalibrationActive = false;
  saveImuCalibration();
  centerAimTracking(false);

  Serial.println(F("[IMU] Calibration complete and saved."));
  Serial.printf("[IMU] Gyro offsets rad/s: X=%.5f Y=%.5f Z=%.5f\n",
                gyroOffsetX, gyroOffsetY, gyroOffsetZ);
  Serial.printf("[IMU] Accel baseline m/s^2: X=%.3f Y=%.3f Z=%.3f\n",
                accelBaselineX, accelBaselineY, accelBaselineZ);
}

void printCalibrationStatus() {
  Serial.println();
  Serial.println(F("[STATUS] Calibration status"));
  Serial.printf("[STATUS] MPU6500: %s", mpuPresent ? "present" : "missing");
  if (mpuPresent) Serial.printf(" at 0x%02X", mpuAddr);
  Serial.println();

  Serial.printf("[STATUS] IMU calibration: %s%s\n",
                imuCalibrated ? "SAVED" : "not set",
                imuCalibrationActive ? " (running)" : "");
  Serial.printf("[STATUS] Gyro offsets rad/s: X=%.5f Y=%.5f Z=%.5f\n",
                gyroOffsetX, gyroOffsetY, gyroOffsetZ);
  Serial.printf("[STATUS] Accel baseline m/s^2: X=%.3f Y=%.3f Z=%.3f\n",
                accelBaselineX, accelBaselineY, accelBaselineZ);

  Serial.printf("[STATUS] Screen size: %u x %u\n", screenWidth, screenHeight);
  Serial.printf("[STATUS] Screen calibration: %s%s\n",
                screenCalibrated ? "SAVED" : "not set",
                calibrationMode ? " (running)" : "");
  Serial.printf("[STATUS] Aim filtered: X=%.4f Y=%.4f | virtual cursor: X=%.0f Y=%.0f\n",
                filteredAimX, filteredAimY, virtualCursorX, virtualCursorY);
  Serial.printf("[STATUS] IR mode: %s | Serial scroll: %s | Debug: %s\n",
                irModeName(),
                serialScrollMode ? "ON" : "OFF",
                DEBUG_MODE ? "ON" : "OFF");
}

// ===================== Motion =====================
void updateMotionState(ImuEvent &imu, unsigned long now) {
  if (lastAimUpdateTime == 0) {
    lastAimUpdateTime = now;
    return;
  }

  float dt = (now - lastAimUpdateTime) / 1000.0f;
  lastAimUpdateTime = now;

  if (dt <= 0.0f || dt > 0.2f) return;

  float correctedGx = imu.gx - gyroOffsetX;
  float correctedGy = imu.gy - gyroOffsetY;
  float correctedGz = imu.gz - gyroOffsetZ;
  float maxGyro = max(fabsf(correctedGx), max(fabsf(correctedGy), fabsf(correctedGz)));

  bool canIdleLock = !stableIrTouched &&
                     !leftButtonDown &&
                     !calibrationMode &&
                     !serialScrollMode;

  if (canIdleLock && maxGyro < STILL_GYRO_THRESHOLD) {
    if (lastStillStartTime == 0) lastStillStartTime = now;
  } else {
    lastStillStartTime = 0;
    motion.idleLocked = false;
  }

  if (canIdleLock && lastStillStartTime != 0 && now - lastStillStartTime >= STILL_LOCK_MS) {
    motion.idleLocked = true;

    gyroOffsetX = applyEMA(gyroOffsetX, imu.gx, AUTO_ZERO_ALPHA);
    gyroOffsetY = applyEMA(gyroOffsetY, imu.gy, AUTO_ZERO_ALPHA);
    gyroOffsetZ = applyEMA(gyroOffsetZ, imu.gz, AUTO_ZERO_ALPHA);

    motion.cursorRateX = 0.0f;
    motion.cursorRateY = 0.0f;
    smoothX = applyEMA(smoothX, 0.0f, 0.35f);
    smoothY = applyEMA(smoothY, 0.0f, 0.35f);
    cursorXFilter.reset(0.0f);
    cursorYFilter.reset(0.0f);
  } else {
    float rawCursorX = gyroAxis(imu, CURSOR_X_GYRO_AXIS) * CURSOR_X_SIGN;
    float rawCursorY = gyroAxis(imu, CURSOR_Y_GYRO_AXIS) * CURSOR_Y_SIGN;

    rawCursorX = applyDeadzone(rawCursorX, DEADZONE);
    rawCursorY = applyDeadzone(rawCursorY, DEADZONE);

    motion.cursorRateX = cursorXFilter.update(rawCursorX);
    motion.cursorRateY = cursorYFilter.update(rawCursorY);

    aimX += motion.cursorRateX * dt;
    aimY += motion.cursorRateY * dt;
  }

  filteredAimX = aimXFilter.update(aimX);
  filteredAimY = aimYFilter.update(aimY);
  // Use filtered vertical gyro motion for scrolling. Accel-only scroll can become one-sided
  // depending on how the tip-mounted MPU sits relative to gravity.
  float gyroScroll = motion.cursorRateY * SCROLL_GYRO_SCALE * SCROLL_SIGN;
  motion.scrollInput = scrollFilter.update(gyroScroll);
  motion.dropInput = dropFilter.update(correctedAccelAxis(imu, DROP_ACCEL_AXIS) * DROP_SIGN);
}

void handleCursorMovement() {
  if (motion.idleLocked) {
    return;
  }

  float targetX = motion.cursorRateX * GYRO_SENSITIVITY;
  float targetY = motion.cursorRateY * GYRO_SENSITIVITY;

  smoothX = applyEMA(smoothX, targetX, SMOOTH_ALPHA);
  smoothY = applyEMA(smoothY, targetY, SMOOTH_ALPHA);

  if (fabsf(smoothX) < OUTPUT_MIN_STEP) smoothX = 0.0f;
  if (fabsf(smoothY) < OUTPUT_MIN_STEP) smoothY = 0.0f;

  int8_t dx = constrain((int)smoothX, -127, 127);
  int8_t dy = constrain((int)smoothY, -127, 127);

  if (dx != 0 || dy != 0) {
    bleMouse.move(dx, dy, 0);
  }

  if (DEBUG_MODE) printDebug(dx, dy);
}

void handleCalibratedCursorMovement() {
  if (motion.idleLocked) {
    return;
  }

  float targetX = mapFloat(filteredAimX, calLeftX, calRightX, 0.0f, screenWidth - 1.0f);
  float targetY = mapFloat(filteredAimY, calTopY, calBottomY, 0.0f, screenHeight - 1.0f);

  targetX = clampFloat(targetX, 0.0f, screenWidth - 1.0f);
  targetY = clampFloat(targetY, 0.0f, screenHeight - 1.0f);

  float errorX = targetX - virtualCursorX;
  float errorY = targetY - virtualCursorY;

  int dx = (int)(errorX * CALIBRATED_RESPONSE);
  int dy = (int)(errorY * CALIBRATED_RESPONSE);

  if (fabsf(errorX) < 1.0f) dx = 0;
  if (fabsf(errorY) < 1.0f) dy = 0;

  dx = constrain(dx, -MAX_CALIBRATED_STEP, MAX_CALIBRATED_STEP);
  dy = constrain(dy, -MAX_CALIBRATED_STEP, MAX_CALIBRATED_STEP);

  if (dx != 0 || dy != 0) {
    bleMouse.move((int8_t)dx, (int8_t)dy, 0);
    virtualCursorX = clampFloat(virtualCursorX + dx, 0.0f, screenWidth - 1.0f);
    virtualCursorY = clampFloat(virtualCursorY + dy, 0.0f, screenHeight - 1.0f);
  }

  if (DEBUG_MODE) {
    static unsigned long lastPrint = 0;
    unsigned long now = millis();

    if (now - lastPrint > 100) {
      lastPrint = now;
      Serial.printf("[ABS] aim=(%.3f,%.3f) virtual=(%.0f,%.0f) move=(%d,%d)\n",
                    filteredAimX, filteredAimY, virtualCursorX, virtualCursorY, dx, dy);
    }
  }
}

// ===================== IR / Click / Drag / Scroll =====================
void handleIRInput() {
  unsigned long now = millis();

  bool rawTouched = IR_ACTIVE_LOW
                      ? (digitalRead(IR_CLICK_PIN) == LOW)
                      : (digitalRead(IR_CLICK_PIN) == HIGH);

  if (rawTouched != lastRawIrTouched) {
    lastRawIrTouched = rawTouched;
    lastIrChangeTime = now;
  }

  if (now - lastIrChangeTime >= IR_DEBOUNCE_MS) {
    stableIrTouched = rawTouched;
  }

  bool pressedEdge = stableIrTouched && !previousStableIrTouched;
  bool releasedEdge = !stableIrTouched && previousStableIrTouched;

  if (pressedEdge) {
    irPressStartTime = now;
    irMaxGyroMotion = 0.0f;
    irMaxAccelMotion = 0.0f;
    irMotionExceededForCalibration = false;
    suppressClickUntilRelease = false;
    irMode = IR_PENDING;

    if (calibrationMode) {
      captureCalibrationCorner();
      suppressClickUntilRelease = true;
      irMode = IR_CALIBRATION_CAPTURE;
    }
  }

  if (calibrationMode) {
    if (releasedEdge) {
      suppressClickUntilRelease = false;
      irMode = IR_IDLE;
    }
    previousStableIrTouched = stableIrTouched;
    return;
  }

  if (stableIrTouched && irMode == IR_PENDING) {
    unsigned long heldFor = now - irPressStartTime;

    if (!irMotionExceededForCalibration && heldFor >= CALIBRATION_HOLD_MS) {
      releaseAllMouseButtons();
      suppressClickUntilRelease = true;
      lastClickOrDropTime = now;
      irMode = IR_CALIBRATION_CAPTURE;
      startScreenCalibration();
      previousStableIrTouched = stableIrTouched;
      return;
    }

    if (heldFor >= SCROLL_HOLD_ARM_MS && fabsf(motion.scrollInput) > SCROLL_DEADZONE) {
      irMode = IR_SCROLLING;
      irMotionExceededForCalibration = true;
      scrollAccumulator = 0.0f;
      if (DEBUG_MODE) Serial.println(F("[SCROLL] IR hold scroll mode"));
    } else if (heldFor >= HOLD_DRAG_MS) {
      pressLeftButton();
      dragStartTime = now;
      suppressClickUntilRelease = true;
      irMode = IR_DRAGGING;
      if (DEBUG_MODE) Serial.println(F("[IR] DRAG START"));
    }
  }

  if (releasedEdge) {
    unsigned long pressDuration = now - irPressStartTime;
    bool cleanTapMotion = irMaxGyroMotion < CLICK_CANCEL_GYRO;
    bool cleanTapDuration = pressDuration >= CLICK_MIN_PRESS_MS &&
                            pressDuration <= CLICK_MAX_PRESS_MS;

    if (leftButtonDown || irMode == IR_DRAGGING) {
      releaseLeftButton();
      lastClickOrDropTime = now;
      if (DEBUG_MODE) Serial.println(F("[IR] DRAG END"));
    } else if (irMode == IR_PENDING &&
               !suppressClickUntilRelease &&
               cleanTapMotion &&
               cleanTapDuration &&
               now - lastIrClickTime >= IR_CLICK_COOLDOWN_MS &&
               now - lastClickOrDropTime >= SCROLL_AFTER_ACTION_LOCKOUT_MS) {
      bleMouse.click(MOUSE_LEFT);
      lastIrClickTime = now;
      lastClickOrDropTime = now;
      scrollAccumulator = 0.0f;
      if (DEBUG_MODE) Serial.println(F("[IR] LEFT CLICK"));
    } else if (DEBUG_MODE && irMode == IR_PENDING) {
      Serial.printf("[IR] tap ignored duration=%lu gyro=%.3f\n",
                    pressDuration, irMaxGyroMotion);
    }

    suppressClickUntilRelease = false;
    scrollAccumulator = 0.0f;
    irMode = IR_IDLE;
  }

  previousStableIrTouched = stableIrTouched;
}

bool handleScrolling() {
  unsigned long now = millis();

  if (calibrationMode || imuCalibrationActive || leftButtonDown) return false;

  bool scrollActive = serialScrollMode || irMode == IR_SCROLLING;
  if (!scrollActive) return false;

  if (now - lastClickOrDropTime < SCROLL_AFTER_ACTION_LOCKOUT_MS) return true;
  if (now - lastScrollTime < SCROLL_COOLDOWN_MS) return true;

  float proportional = applyDeadzone(motion.scrollInput, SCROLL_DEADZONE) * SCROLL_SENSITIVITY;

  if (fabsf(proportional) < 0.01f) {
    scrollAccumulator *= 0.80f;
    return true;
  }

  scrollAccumulator += proportional;
  scrollAccumulator = clampFloat(scrollAccumulator, -SCROLL_MAX_STEP, SCROLL_MAX_STEP);

  int step = (int)scrollAccumulator;
  if (step == 0) return true;

  step = constrain(step, -SCROLL_MAX_STEP, SCROLL_MAX_STEP);
  scrollAccumulator -= step;

  bleMouse.move(0, 0, (int8_t)step);
  lastScrollTime = now;

  if (DEBUG_MODE) {
    Serial.printf("[SCROLL] input=%.2f step=%d mode=%s\n",
                  motion.scrollInput, step, serialScrollMode ? "serial" : "IR");
  }

  return true;
}

void handleDropGesture() {
  if (!leftButtonDown || irMode != IR_DRAGGING) return;

  unsigned long now = millis();
  if (now - dragStartTime < DROP_MIN_DRAG_MS) return;
  if (now - lastDropGestureTime < DROP_GESTURE_COOLDOWN_MS) return;

  if (motion.dropInput > DROP_GESTURE_THRESHOLD) {
    releaseLeftButton();
    irMode = IR_DROPPED;
    suppressClickUntilRelease = true;
    lastDropGestureTime = now;
    lastClickOrDropTime = now;
    scrollAccumulator = 0.0f;

    if (DEBUG_MODE) Serial.println(F("[DROP] Released drag"));
  }
}

void updateIrMotionGuard(ImuEvent &imu) {
  if (!stableIrTouched || calibrationMode || imuCalibrationActive) return;

  float gx = fabsf(imu.gx - gyroOffsetX);
  float gy = fabsf(imu.gy - gyroOffsetY);
  float gz = fabsf(imu.gz - gyroOffsetZ);
  float maxGyro = max(gx, max(gy, gz));

  float ax = fabsf(correctedAccelX(imu));
  float ay = fabsf(correctedAccelY(imu));
  float az = fabsf(correctedAccelZ(imu));
  float maxAccel = max(ax, max(ay, az));

  if (maxGyro > irMaxGyroMotion) irMaxGyroMotion = maxGyro;
  if (maxAccel > irMaxAccelMotion) irMaxAccelMotion = maxAccel;

  if (maxGyro > CALIBRATION_HOLD_GYRO_LIMIT ||
      maxAccel > CALIBRATION_HOLD_ACCEL_LIMIT) {
    irMotionExceededForCalibration = true;
  }
}

void pressLeftButton() {
  if (!leftButtonDown) {
    bleMouse.press(MOUSE_LEFT);
    leftButtonDown = true;
  }
}

void releaseLeftButton() {
  if (leftButtonDown) {
    bleMouse.release(MOUSE_LEFT);
    leftButtonDown = false;
  }
}

void releaseAllMouseButtons() {
  if (bleMouse.isConnected()) {
    bleMouse.release(MOUSE_LEFT);
    bleMouse.release(MOUSE_RIGHT);
    bleMouse.release(MOUSE_MIDDLE);
  }
  leftButtonDown = false;
}

// ===================== MPU6500 Low Level =====================
void scanI2CBus() {
  Serial.println(F("[I2C] Scanning SDA=GPIO21 SCL=GPIO22..."));

  byte count = 0;
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0) {
      Serial.printf("[I2C] Found device at 0x%02X\n", address);
      count++;
    }

    delay(2);
  }

  if (count == 0) {
    Serial.println(F("[I2C] No devices found. Check SDA/SCL/power/GND/CS wiring."));
  }
}

bool mpu6500Begin() {
  if (mpu6500BeginAt(0x68)) return true;
  if (mpu6500BeginAt(0x69)) return true;
  return false;
}

bool mpu6500BeginAt(uint8_t address) {
  mpuAddr = address;
  uint8_t id = readReg(WHO_AM_I);

  Serial.printf("[MPU] Address 0x%02X WHO_AM_I = 0x%02X\n", address, id);

  if (id != 0x70 && id != 0x71 && id != 0x73 && id != 0x68) {
    return false;
  }

  writeReg(PWR_MGMT_1, 0x00);
  delay(100);

  writeReg(CONFIG_REG, 0x04);      // DLPF approx 20 Hz
  writeReg(GYRO_CONFIG, 0x08);     // +/-500 deg/s
  writeReg(ACCEL_CONFIG, 0x10);    // +/-8g

  return true;
}

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(mpuAddr, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

void readMPU6500(ImuEvent &imu) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);

  uint8_t received = Wire.requestFrom(mpuAddr, (uint8_t)14);
  if (received < 14) {
    imu.ax = imu.ay = imu.az = 0.0f;
    imu.gx = imu.gy = imu.gz = 0.0f;
    return;
  }

  int16_t axRaw = (Wire.read() << 8) | Wire.read();
  int16_t ayRaw = (Wire.read() << 8) | Wire.read();
  int16_t azRaw = (Wire.read() << 8) | Wire.read();

  Wire.read();
  Wire.read();

  int16_t gxRaw = (Wire.read() << 8) | Wire.read();
  int16_t gyRaw = (Wire.read() << 8) | Wire.read();
  int16_t gzRaw = (Wire.read() << 8) | Wire.read();

  imu.ax = axRaw * ACCEL_SCALE;
  imu.ay = ayRaw * ACCEL_SCALE;
  imu.az = azRaw * ACCEL_SCALE;

  imu.gx = gxRaw * GYRO_SCALE;
  imu.gy = gyRaw * GYRO_SCALE;
  imu.gz = gzRaw * GYRO_SCALE;
}

// ===================== Helpers =====================
void resetMotionFilters() {
  cursorXFilter.reset(0.0f);
  cursorYFilter.reset(0.0f);
  aimXFilter.reset(aimX);
  aimYFilter.reset(aimY);
  scrollFilter.reset(0.0f);
  dropFilter.reset(0.0f);
  motion.idleLocked = false;
}

float gyroAxis(ImuEvent &imu, uint8_t axis) {
  if (axis == 0) return imu.gx - gyroOffsetX;
  if (axis == 1) return imu.gy - gyroOffsetY;
  return imu.gz - gyroOffsetZ;
}

float accelAxis(ImuEvent &imu, uint8_t axis) {
  if (axis == 0) return imu.ax;
  if (axis == 1) return imu.ay;
  return imu.az;
}

float correctedAccelX(ImuEvent &imu) {
  return imuCalibrated ? (imu.ax - accelBaselineX) : imu.ax;
}

float correctedAccelY(ImuEvent &imu) {
  return imuCalibrated ? (imu.ay - accelBaselineY) : imu.ay;
}

float correctedAccelZ(ImuEvent &imu) {
  return imuCalibrated ? (imu.az - accelBaselineZ) : imu.az;
}

float correctedAccelAxis(ImuEvent &imu, uint8_t axis) {
  if (axis == 0) return correctedAccelX(imu);
  if (axis == 1) return correctedAccelY(imu);
  return correctedAccelZ(imu);
}

float mapFloat(float value, float inMin, float inMax, float outMin, float outMax) {
  if (fabsf(inMax - inMin) < 0.000001f) return outMin;
  return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

float clampFloat(float value, float low, float high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

float applyDeadzone(float value, float threshold) {
  if (fabsf(value) < threshold) return 0.0f;
  return (value > 0) ? (value - threshold) : (value + threshold);
}

float applyEMA(float previous, float current, float alpha) {
  return alpha * current + (1.0f - alpha) * previous;
}

const char *irModeName() {
  switch (irMode) {
    case IR_IDLE: return "IDLE";
    case IR_PENDING: return "PENDING";
    case IR_SCROLLING: return "SCROLLING";
    case IR_DRAGGING: return "DRAGGING";
    case IR_DROPPED: return "DROPPED";
    case IR_CALIBRATION_CAPTURE: return "CAL_CAPTURE";
  }
  return "UNKNOWN";
}

void printDebug(float dx, float dy) {
  static unsigned long lastPrint = 0;
  unsigned long now = millis();

  if (now - lastPrint < 120) return;
  lastPrint = now;

  Serial.printf("[MOVE] rateX=%6.3f rateY=%6.3f scroll=%6.2f drop=%6.2f idle=%d IR=%s | dx=%4d dy=%4d\n",
                motion.cursorRateX,
                motion.cursorRateY,
                motion.scrollInput,
                motion.dropInput,
                motion.idleLocked ? 1 : 0,
                irModeName(),
                (int)dx,
                (int)dy);
}

/*
 * Adaptive PID Line Following Robot
 * ---------------------------------
 * Controller : Arduino Nano
 * Motor Driver: TB6612FNG
 * Sensors    : 8-channel analog reflectance array
 *
 * Features:
 * - Automatic sensor calibration
 * - PID-based line tracking
 * - Adaptive Kp/Kd for straight sections and sharp turns
 * - PID output smoothing
 * - Speed reduction during large tracking errors
 * - Controlled line-loss recovery
 * - Short stabilization phase after recovery
 * - Straight-through intersection handling
 * - Single-button state machine
 *
 * NOTE:
 * Tune Kp, Ki, Kd and the calibration threshold for your
 * specific sensor array, motor gearing, battery voltage and track.
 */

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// -------------------- TB6612FNG pins --------------------
constexpr uint8_t AIN1 = 4;
constexpr uint8_t AIN2 = 3;
constexpr uint8_t BIN1 = 6;
constexpr uint8_t BIN2 = 7;
constexpr uint8_t PWMA = 9;
constexpr uint8_t PWMB = 10;
constexpr uint8_t STBY = 5;

// -------------------- User interface --------------------
constexpr uint8_t BUTTON_PIN = 11;
constexpr uint8_t LED_PIN = 13;

// -------------------- Operating states --------------------
enum class RobotMode : uint8_t {
  IDLE,
  CALIBRATION,
  READY,
  FINAL_RUN
};

RobotMode mode = RobotMode::IDLE;

// -------------------- PID tuning --------------------
constexpr float KP = 17.0f;
constexpr float KI = 0.0f;
constexpr float KD = 150.0f;

float proportionalTerm = 0.0f;
float integralTerm = 0.0f;
float derivativeTerm = 0.0f;

float error = 0.0f;
float previousError = 0.0f;
float lastValidError = 0.0f;

// -------------------- Speed control --------------------
constexpr int MIN_SPEED = 80;
constexpr int FINAL_MAX_SPEED = 240;

// -------------------- Sensor configuration --------------------
constexpr uint8_t SENSOR_COUNT = 8;
const int8_t SENSOR_WEIGHT[SENSOR_COUNT] = {7, 5, 3, 1, -1, -3, -5, -7};

int sensorRaw[SENSOR_COUNT];
bool sensorActive[SENSOR_COUNT];

int sensorMin[SENSOR_COUNT];
int sensorMax[SENSOR_COUNT];

constexpr int LINE_THRESHOLD = 250;
constexpr int CALIBRATION_PADDING = 20;

// -------------------- Runtime state --------------------
bool onLine = false;
bool intersectionDetected = false;
bool recoveringFromLineLoss = false;

unsigned long calibrationStart = 0;
unsigned long lineLossStart = 0;
unsigned long turnExitTime = 0;

// -------------------- Setup --------------------
void setup() {
  // Increase ADC sampling speed.
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  digitalWrite(STBY, HIGH);

  // Timer1 prescaler = 1 for higher PWM frequency on pins 9/10.
  TCCR1B = (TCCR1B & 0b11111000) | 0b00000001;

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    sensorMin[i] = 1023;
    sensorMax[i] = 0;
  }

  Serial.begin(115200);
}

// -------------------- Main loop --------------------
void loop() {
  handleButton();
  updateStatusLED();

  switch (mode) {
    case RobotMode::CALIBRATION:
      runCalibration();
      break;

    case RobotMode::READY:
      stopMotors();
      break;

    case RobotMode::FINAL_RUN:
      followLine(FINAL_MAX_SPEED);
      break;

    case RobotMode::IDLE:
    default:
      stopMotors();
      break;
  }
}

// -------------------- Button/state machine --------------------
void handleButton() {
  static bool previousButtonState = HIGH;
  const bool currentButtonState = digitalRead(BUTTON_PIN);

  // Detect falling edge because INPUT_PULLUP is used.
  if (previousButtonState == HIGH && currentButtonState == LOW) {
    delay(50);  // Simple debounce.

    if (mode == RobotMode::IDLE) {
      delay(200);
      mode = RobotMode::CALIBRATION;
      calibrationStart = millis();
    } 
    else if (mode == RobotMode::READY) {
      delay(200);
      mode = RobotMode::FINAL_RUN;
    } 
    else if (mode == RobotMode::FINAL_RUN ||
             mode == RobotMode::CALIBRATION) {
      // Emergency stop / reset.
      mode = RobotMode::IDLE;
      stopMotors();
    }
  }

  previousButtonState = currentButtonState;
}

// -------------------- Sensor calibration --------------------
void runCalibration() {
  // Rotate in place so all sensors observe the track surface.
  setMotorSpeed(90, -90);

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    const int value = analogRead(i);

    if (value < sensorMin[i]) sensorMin[i] = value;
    if (value > sensorMax[i]) sensorMax[i] = value;
  }

  if (millis() - calibrationStart > 4000UL) {
    stopMotors();

    // Add margin around measured extremes to widen the usable window.
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
      sensorMin[i] += CALIBRATION_PADDING;
      sensorMax[i] -= CALIBRATION_PADDING;

      // Prevent an invalid calibration range.
      if (sensorMax[i] <= sensorMin[i]) {
        sensorMax[i] = sensorMin[i] + 1;
      }
    }

    mode = RobotMode::READY;
  }
}

// -------------------- Line following --------------------
void followLine(int baseSpeed) {
  readLineSensors();

  // ---------------- Line-loss recovery ----------------
  if (!onLine) {
    if (!recoveringFromLineLoss) {
      recoveringFromLineLoss = true;
      lineLossStart = millis();
    }

    // Continue turning in the direction of the last valid error.
    if (lastValidError > 0) {
      setMotorSpeed(180, -180);
    } else {
      setMotorSpeed(-180, 180);
    }

    turnExitTime = millis();

    // Minimum turn commitment.
    if (millis() - lineLossStart < 120UL) {
      return;
    }

    return;
  }

  // ---------------- Position/error estimation ----------------
  long weightedSum = 0;
  int activeSensorCount = 0;

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    if (sensorActive[i]) {
      weightedSum += SENSOR_WEIGHT[i];
      ++activeSensorCount;
    }
  }

  if (activeSensorCount > 0) {
    error = static_cast<float>(weightedSum) / activeSensorCount;
  }

  recoveringFromLineLoss = false;

  // Short stabilization period after a recovery turn.
  if (millis() - turnExitTime < 120UL) {
    setMotorSpeed(120, 120);
    return;
  }

  // ---------------- Straight-through intersection ----------------
  if (activeSensorCount >= 5 &&
      sensorActive[0] &&
      sensorActive[7]) {
    if (!intersectionDetected) {
      driveStraightThroughIntersection();
      intersectionDetected = true;
    }
  } else {
    intersectionDetected = false;
  }

  // ---------------- PID calculation ----------------
  proportionalTerm = error;

  if (abs(error) < 2.0f) {
    integralTerm += error;
  } else {
    integralTerm = 0.0f;
  }

  integralTerm = constrain(integralTerm, -30.0f, 30.0f);
  derivativeTerm = error - previousError;

  float dynamicKp = KP;
  float dynamicKd = KD;

  // Reduce oscillation when centered on a straight section.
  if (abs(error) < 1.5f) {
    dynamicKp *= 0.5f;
    dynamicKd *= 0.6f;
  }
  // Increase response during larger deviations.
  else if (abs(error) > 4.0f) {
    dynamicKp *= 1.2f;
    dynamicKd *= 1.3f;
  }

  const float rawPID =
      (dynamicKp * proportionalTerm) +
      (KI * integralTerm) +
      (dynamicKd * derivativeTerm);

  // Exponential smoothing reduces abrupt motor commands.
  static float smoothedPID = 0.0f;
  smoothedPID = (0.7f * smoothedPID) + (0.3f * rawPID);

  const float pidOutput = constrain(smoothedPID, -250.0f, 250.0f);

  previousError = error;

  if (abs(error) > 0.5f) {
    lastValidError = error;
  }

  // Slow down for large deviations to reduce overshoot.
  int currentBaseSpeed = baseSpeed;

  if (abs(error) > 5.5f) {
    currentBaseSpeed = static_cast<int>(baseSpeed * 0.3f);
  } 
  else if (abs(error) > 3.5f) {
    currentBaseSpeed = static_cast<int>(baseSpeed * 0.6f);
  }

  currentBaseSpeed = constrain(currentBaseSpeed, MIN_SPEED, baseSpeed);

  setMotorSpeed(
      static_cast<int>(currentBaseSpeed + pidOutput),
      static_cast<int>(currentBaseSpeed - pidOutput)
  );
}

// -------------------- Sensor processing --------------------
void readLineSensors() {
  onLine = false;

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    sensorRaw[i] = analogRead(i);

    int calibratedValue =
        map(sensorRaw[i], sensorMin[i], sensorMax[i], 0, 1000);

    calibratedValue = constrain(calibratedValue, 0, 1000);

    sensorActive[i] = calibratedValue > LINE_THRESHOLD;

    if (sensorActive[i]) {
      onLine = true;
    }
  }
}

// -------------------- Status LED --------------------
void updateStatusLED() {
  switch (mode) {
    case RobotMode::IDLE:
      digitalWrite(LED_PIN, LOW);
      break;

    case RobotMode::CALIBRATION:
      digitalWrite(LED_PIN, (millis() % 400UL) < 200UL);
      break;

    case RobotMode::READY:
      digitalWrite(LED_PIN, (millis() % 120UL) < 60UL);
      break;

    case RobotMode::FINAL_RUN:
      digitalWrite(LED_PIN, (millis() % 2000UL) < 1000UL);
      break;
  }
}

// -------------------- Motor control --------------------
void setMotorSpeed(int left, int right) {
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  digitalWrite(AIN1, left >= 0);
  digitalWrite(AIN2, left < 0);
  analogWrite(PWMA, abs(left));

  digitalWrite(BIN1, right >= 0);
  digitalWrite(BIN2, right < 0);
  analogWrite(PWMB, abs(right));
}

void driveStraightThroughIntersection() {
  setMotorSpeed(250, 250);
  delay(60);
}

void stopMotors() {
  // TB6612FNG short-brake state.
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, HIGH);
}

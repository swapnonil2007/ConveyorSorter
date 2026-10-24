/*
  Automatic conveyor-belt length sorter
  Board: Arduino UNO

  Wiring:
    PING))) SIG  -> D12
    Servo signal -> D11
    MOSFET gate   -> D3 through 100-220 ohm resistor
    10k resistor -> MOSFET Gate to Source/GND
    All grounds common.
*/

#include <Servo.h>

const uint8_t PING_PIN  = 12;  
const uint8_t SERVO_PIN = 11;
const uint8_t MOTOR_PIN = 3;  

// Conveyor settings
const uint8_t MOTOR_PWM = 110;       // 0-255; tune for a slow belt
const float BELT_SPEED_CM_S = 4.0;   // CALIBRATE with a ruler and stopwatch

// Object detection
// Sensor should see EMPTY belt/background outside this distance window,
// and the object within it while passing the sensing line.
const float DETECT_MIN_CM = 5.0;
const float DETECT_MAX_CM = 12.0;
const unsigned long PING_INTERVAL_MS = 30;
const uint8_t HIT_LIMIT  = 2;  // reject single noisy readings
const uint8_t MISS_LIMIT = 3;  // end object after this many missed readings
const float MIN_VALID_LENGTH_CM = 0.5;

// Length classes
const float LONG_MIN_CM   = 8.0; // >= 8 cm: straight off the end; servo stays open
const float MEDIUM_MIN_CM = 4.0; // >= 4 and < 8 cm: 30 degrees
                                  // < 4 cm: 150 degrees

// Servo geometry
const int ANGLE_OPEN   = 0;
const int ANGLE_MEDIUM = 30;
const int ANGLE_SHORT  = 150;

// Measure along the belt from the sensor's sensing line to the blade.
// It should exceed the maximum object length.
const float SENSOR_TO_SERVO_CM = 15.0;
const float CLEAR_MARGIN_CM = 3.0;

// true: move the blade as soon as the object's length is estimated.
// This gives the servo time to move before the object reaches the blade.
// false: schedule movement for when the object's front reaches the blade;
// this can be too late if the servo needs time to move.
const bool SERVO_ACT_IMMEDIATELY = true;

// State
Servo sorter;

unsigned long lastPingMs = 0;

bool inObject = false;
uint8_t hitStreak = 0;
uint8_t missStreak = 0;
unsigned long candidateStartMs = 0;
unsigned long objectStartMs = 0;
unsigned long lastHitMs = 0;

bool actionPending = false;
bool deflected = false;
int pendingAngle = ANGLE_OPEN;
unsigned long actionAtMs = 0;
unsigned long releaseAtMs = 0;

// Helpers
float readDistanceCm() {
  // PING))) requires a trigger pulse, then switches the same SIG pin to input.
  pinMode(PING_PIN, OUTPUT);
  digitalWrite(PING_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(PING_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(PING_PIN, LOW);

  pinMode(PING_PIN, INPUT);
  const unsigned long echoUs = pulseIn(PING_PIN, HIGH, 30000UL);
  if (echoUs == 0) return -1.0; // no echo / timeout

  return echoUs / 58.0;         // approximate distance in cm
}

bool timeReached(unsigned long now, unsigned long target) {
  return (long)(now - target) >= 0; 
}

void handleFinishedObject(unsigned long now) {
  // Estimate how long the object occupied the sensor's detection window.
  const unsigned long detectedMs =
      (lastHitMs - objectStartMs) + PING_INTERVAL_MS;
  const float durationS = detectedMs / 1000.0;
  const float lengthCm = durationS * BELT_SPEED_CM_S;

  Serial.print(F("Estimated object length: "));
  Serial.print(lengthCm, 1);
  Serial.println(F(" cm"));

  if (lengthCm < MIN_VALID_LENGTH_CM) {
    Serial.println(F("Ignored: likely noise or too small to measure reliably."));
    return;
  }

  if (lengthCm >= LONG_MIN_CM) {
    Serial.println(F("Class: LONG (>= 8 cm) -> servo remains open"));
    // Do not move the blade for long objects.
    return;
  }

  if (lengthCm >= MEDIUM_MIN_CM) {
    pendingAngle = ANGLE_MEDIUM;
    Serial.println(F("Class: MEDIUM (4 to <8 cm) -> servo 30 degrees"));
  } else {
    pendingAngle = ANGLE_SHORT;
    Serial.println(F("Class: SHORT (<4 cm) -> servo 150 degrees"));
  }

  unsigned long delayMs = 0;
  if (!SERVO_ACT_IMMEDIATELY && SENSOR_TO_SERVO_CM > lengthCm) {
    delayMs = (unsigned long)(
      ((SENSOR_TO_SERVO_CM - lengthCm) / BELT_SPEED_CM_S) * 1000.0
    );
  }

  actionAtMs = now + delayMs;

  // After the object's tail has travelled past the blade plus a clearance margin,
  // return the blade to the open position.
  releaseAtMs = now + (unsigned long)(
    ((SENSOR_TO_SERVO_CM + CLEAR_MARGIN_CM) / BELT_SPEED_CM_S) * 1000.0
  );

  actionPending = true;
}

void updateServo(unsigned long now) {
  if (actionPending && timeReached(now, actionAtMs)) {
    sorter.write(pendingAngle);
    actionPending = false;
    deflected = true;
  }

  if (deflected && !actionPending && timeReached(now, releaseAtMs)) {
    sorter.write(ANGLE_OPEN);
    deflected = false;
  }
}

// Arduino entry points
void setup() {
  Serial.begin(9600);

  sorter.attach(SERVO_PIN);
  sorter.write(ANGLE_OPEN);

  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, LOW);

  
  
  analogWrite(MOTOR_PIN, 255);
  delay(250);
  analogWrite(MOTOR_PIN, MOTOR_PWM);

  Serial.println(F("Conveyor length sorter ready."));
  Serial.println(F("Open Serial Monitor at 9600 baud."));
  Serial.println(F("Calibrate BELT_SPEED_CM_S before sorting."));
}

void loop() {
  const unsigned long now = millis();

  updateServo(now);

  if (now - lastPingMs < PING_INTERVAL_MS) return;
  lastPingMs = now;

  const float distanceCm = readDistanceCm();
  const bool hit = (distanceCm >= DETECT_MIN_CM &&
                    distanceCm <= DETECT_MAX_CM);

  if (hit) {
    missStreak = 0;
    lastHitMs = now;

    if (!inObject) {
      if (hitStreak == 0) candidateStartMs = now;
      hitStreak++;

      if (hitStreak >= HIT_LIMIT) {
        inObject = true;
        objectStartMs = candidateStartMs;
      }
    }
  } else {
    if (inObject) {
      missStreak++;

      if (missStreak >= MISS_LIMIT) {
        inObject = false;
        missStreak = 0;
        hitStreak = 0;
        handleFinishedObject(now);
      }
    } else {
      hitStreak = 0;
    }
  }
}

#include <AccelStepper.h>

/* ===============================
   DRIVER
   =============================== */
const int stepPin = 2;
const int dirPin  = 3;
const int enablePin = 4;

AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

/* ===============================
   INPUTS / OUTPUTS
   =============================== */
const int closeBtn = 7;
const int closeLED = 6;

const int openBtn  = 48;
const int openLED  = 49;

const int openStopLimit  = 47;
const int closeStopLimit = 46;

/* ===============================
   MOTION PROFILE (UNCHANGED)
   =============================== */
float cruiseSpeed = 6000;
float slowSpeed   = 1600;
float minSpeed    = 1200;

/* ===============================
   CALIBRATION PROFILE (UNCHANGED)
   =============================== */
const float CAL_SPEED = 1200;

/* ===============================
   TRAVEL MODEL
   =============================== */
long fullTravelSteps = 0;
long rampZoneSteps = 0;
long stopZoneSteps = 0;

/* ===============================
   STATE
   =============================== */
enum Cycle { NONE, OPEN, CLOSE };
Cycle activeCycle = NONE;

/* ===============================
   BUTTON EDGE TRACKING (optional stability)
   =============================== */
bool lastOpenBtn = HIGH;
bool lastCloseBtn = HIGH;

/* ===============================
   SETUP
   =============================== */
void setup() {

  Serial.begin(115200);

  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, LOW);

  pinMode(openBtn, INPUT_PULLUP);
  pinMode(closeBtn, INPUT_PULLUP);

  pinMode(openStopLimit, INPUT_PULLUP);
  pinMode(closeStopLimit, INPUT_PULLUP);

  pinMode(openLED, OUTPUT);
  pinMode(closeLED, OUTPUT);

  digitalWrite(openLED, LOW);
  digitalWrite(closeLED, LOW);

  /* 🔇 QUIET FIX #1: pulse width */
  stepper.setMinPulseWidth(2);

  stepper.setMaxSpeed(cruiseSpeed);
  stepper.setAcceleration(3000);

  calibrateSystem();
  resetMotionState();
}

/* =============================== */
void resetMotionState() {
  stepper.setSpeed(0);
  stepper.setMaxSpeed(cruiseSpeed);
  stepper.setAcceleration(3000);
}

/* ===============================
   CALIBRATION (UNCHANGED)
   =============================== */
void calibrateSystem() {

  Serial.println("CALIBRATION START");

  stepper.setMaxSpeed(CAL_SPEED);

  stepper.setSpeed(-CAL_SPEED);
  while (digitalRead(closeStopLimit) == HIGH) {
    stepper.runSpeed();
  }

  stepper.setSpeed(0);
  delay(50);

  stepper.setCurrentPosition(0);
  delay(200);

  stepper.setSpeed(CAL_SPEED);
  while (digitalRead(openStopLimit) == HIGH) {
    stepper.runSpeed();
  }

  stepper.setSpeed(0);

  fullTravelSteps = stepper.currentPosition();

  rampZoneSteps = fullTravelSteps * 0.2;
  stopZoneSteps = fullTravelSteps * 0.05;

  Serial.print("Full travel: ");
  Serial.println(fullTravelSteps);

  Serial.println("Calibration complete");
}

/* ===============================
   BUTTONS (edge detect = more reliable)
   =============================== */
void handleButtons() {

  bool openNow = digitalRead(openBtn);
  bool closeNow = digitalRead(closeBtn);

  if (lastOpenBtn == HIGH && openNow == LOW && activeCycle == NONE) {
    activeCycle = OPEN;
    digitalWrite(openLED, HIGH);
    digitalWrite(closeLED, LOW);
  }

  if (lastCloseBtn == HIGH && closeNow == LOW && activeCycle == NONE) {
    activeCycle = CLOSE;
    digitalWrite(closeLED, HIGH);
    digitalWrite(openLED, LOW);
  }

  lastOpenBtn = openNow;
  lastCloseBtn = closeNow;
}

/* ===============================
   MOTION + RAMP (QUIET)
   =============================== */
void runMotion(int dir, bool limitSwitch) {

  long pos = stepper.currentPosition();

  long remaining = (dir == 1)
    ? fullTravelSteps - pos
    : pos;

  float speed = cruiseSpeed;

  if (remaining < rampZoneSteps && remaining > stopZoneSteps) {

    float t = (float)(remaining - stopZoneSteps) /
              (float)(rampZoneSteps - stopZoneSteps);

    t = constrain(t, 0, 1);
    float eased = t * t * (3 - 2 * t);

    speed = slowSpeed + (cruiseSpeed - slowSpeed) * eased;
  }

  if (remaining <= stopZoneSteps) {

    float t = (float)remaining / (float)stopZoneSteps;
    t = constrain(t, 0, 1);
    float eased = t * t * (3 - 2 * t);

    speed = slowSpeed * eased;
  }

  if (speed < minSpeed) speed = minSpeed;

  /* 🔇 QUIET FIX #2: micro dithering */
  float dither = sin(stepper.currentPosition() * 0.05) * (speed * 0.003);  // very small

  stepper.setSpeed(dir * (speed + dither));
  stepper.runSpeed();

  if (limitSwitch) {
    stepper.setSpeed(0);
    activeCycle = NONE;

    digitalWrite(openLED, LOW);
    digitalWrite(closeLED, LOW);
  }
}

/* ===============================
   LOOP
   =============================== */
void loop() {

  handleButtons();

  if (activeCycle == OPEN) {
    runMotion(+1, digitalRead(openStopLimit) == LOW);
  }

  if (activeCycle == CLOSE) {
    runMotion(-1, digitalRead(closeStopLimit) == LOW);
  }
}
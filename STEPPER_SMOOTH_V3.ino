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
   MOTION (UNCHANGED)
   =============================== */
float maxSpeed = 30000;
float accel    = 3000;
const float CAL_SPEED = 1500;

/* ===============================
   HOMING
   =============================== */
const int HOMING_BACKOFF_STEPS = 300;
const float HOMING_SLOW_SPEED = 1200;

/* ===============================
   STATE
   =============================== */
bool calibrating = true;
int calStage = 0;

bool motionArmed = false;
bool moving = false;

long fullTravelSteps = 0;

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

  /* 🔇 QUIET FIX: cleaner step edges */
  stepper.setMinPulseWidth(4);

  stepper.setMaxSpeed(maxSpeed);
  stepper.setAcceleration(accel);

  Serial.println("BOOT OK");
}

/* ===============================
   CALIBRATION (UNCHANGED LOGIC)
   =============================== */
void handleCalibration() {

  if (!calibrating) return;

  if (calStage == 0) {

    stepper.setMaxSpeed(CAL_SPEED);
    stepper.setSpeed(-CAL_SPEED);
    stepper.runSpeed();

    if (digitalRead(closeStopLimit) == LOW) {

      stepper.setSpeed(0);
      delay(100);

      stepper.move(HOMING_BACKOFF_STEPS);
      while (stepper.distanceToGo() != 0) {
        stepper.run();
      }

      delay(100);

      stepper.setSpeed(-HOMING_SLOW_SPEED);

      while (digitalRead(closeStopLimit) == HIGH) {
        stepper.runSpeed();
      }

      stepper.setSpeed(0);
      stepper.setCurrentPosition(0);

      calStage = 1;

      Serial.println("CLOSE HOME LOCKED");
    }
  }

  else if (calStage == 1) {

    stepper.setSpeed(CAL_SPEED);
    stepper.runSpeed();

    if (digitalRead(openStopLimit) == LOW) {

      stepper.setSpeed(0);

      fullTravelSteps = stepper.currentPosition();

      stepper.setMaxSpeed(maxSpeed);
      stepper.setAcceleration(accel);

      calibrating = false;
      motionArmed = true;

      stepper.moveTo(stepper.currentPosition());

      Serial.print("FULL TRAVEL = ");
      Serial.println(fullTravelSteps);
      Serial.println("CAL DONE + ARMED");
    }
  }
}

/* ===============================
   BUTTONS
   =============================== */
void handleButtons() {

  if (!motionArmed) return;

  if (digitalRead(openBtn) == LOW) {

    stepper.moveTo(fullTravelSteps);

    moving = true;

    digitalWrite(openLED, HIGH);
    digitalWrite(closeLED, LOW);
  }

  if (digitalRead(closeBtn) == LOW) {

    stepper.moveTo(0);

    moving = true;

    digitalWrite(closeLED, HIGH);
    digitalWrite(openLED, LOW);
  }
}

/* ===============================
   LIMIT SAFETY
   =============================== */
void checkLimits() {

  long pos = stepper.currentPosition();

  if (digitalRead(openStopLimit) == LOW && stepper.targetPosition() > pos) {
    stepper.setCurrentPosition(fullTravelSteps);
    stepper.stop();
  }

  if (digitalRead(closeStopLimit) == LOW && stepper.targetPosition() < pos) {
    stepper.setCurrentPosition(0);
    stepper.stop();
  }
}

/* ===============================
   MOTION COMPLETE
   =============================== */
void updateMotionState() {

  if (moving && stepper.distanceToGo() == 0) {

    moving = false;

    digitalWrite(openLED, LOW);
    digitalWrite(closeLED, LOW);
  }
}

/* ===============================
   🔇 QUIET FIX: eliminate idle buzz
   =============================== */
void quietHold() {
  if (stepper.distanceToGo() == 0) {
    stepper.setSpeed(0);
  }
}

/* ===============================
   LOOP
   =============================== */
void loop() {

  handleCalibration();

  stepper.run();

  quietHold();   // 🔇 noise reduction only

  if (!calibrating) {

    handleButtons();
    checkLimits();
    updateMotionState();
  }
}
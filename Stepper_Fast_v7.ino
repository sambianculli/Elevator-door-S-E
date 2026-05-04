#include <FastAccelStepper.h>
#include <LiquidCrystal_I2C.h>
// microsteps=3200 off off on on
#define numLED 2
#define numBTN 2

// ===============================
// MOTOR PINS
// ===============================
const int stepPin = 6;
const int dirPin  = 7;

// ===============================
// LIMIT SWITCHES
// ===============================
const int openStopLimit  = 50;
const int closeStopLimit = 46;

// ===============================
// BUTTONS / LEDS
// ===============================
int btnPin[numBTN] = {48, 3};
int ledPin[numLED] = {2, 49};

// ===============================
// DEBOUNCE
// ===============================
int buttonState[numBTN] = {HIGH, HIGH};
int lastButtonState[numBTN] = {HIGH, HIGH};
unsigned long lastDebounceTime[numBTN] = {0, 0};
const unsigned long debounceDelay = 50;

// ===============================
// LCD
// ===============================
LiquidCrystal_I2C lcd(0x27, 20, 4);

// ===============================
// STEPPER
// ===============================
FastAccelStepperEngine engine;
FastAccelStepper *stepper = NULL;

// ===============================
// MOTION SETTINGS
// ===============================
int32_t maxSpeed = 5000;
int32_t accel    = 3200;

const int32_t CAL_SPEED = 2000;
const int32_t HOMING_SLOW_SPEED = 600;
const int HOMING_BACKOFF_STEPS = 600;

// ===============================
// STATE
// ===============================
bool calibrating = true;
int calStage = 0;
bool motionArmed = false;
bool moving = false;

int32_t fullTravelSteps = 0;

// ===============================
// SETUP
// ===============================
void setup() {

  Serial.begin(9600);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Position:");
  lcd.setCursor(0,1);
  lcd.print("Speed:");

  pinMode(openStopLimit, INPUT_PULLUP);
  pinMode(closeStopLimit, INPUT_PULLUP);

  for (int i = 0; i < numBTN; i++) {
    pinMode(btnPin[i], INPUT_PULLUP);
  }

  for (int i = 0; i < numLED; i++) {
    pinMode(ledPin[i], OUTPUT);
    digitalWrite(ledPin[i], LOW);
  }

  // Manual enable
  pinMode(8, OUTPUT);
  digitalWrite(8, LOW);

  // Stepper init
  engine.init();
  stepper = engine.stepperConnectToPin(stepPin);

  if (!stepper) {
    Serial.println("STEPPER INIT FAILED");
    while (1);
  }

  stepper->setDirectionPin(dirPin);
  stepper->setAutoEnable(false);

  stepper->setSpeedInHz(maxSpeed);
  stepper->setAcceleration(accel);

  Serial.println("SYSTEM READY - STARTING CALIBRATION");
}

// ===============================
// DEBOUNCE
// ===============================
void debounce() {

  for (int n = 0; n < numBTN; n++) {

    int reading = digitalRead(btnPin[n]);

    if (reading != lastButtonState[n]) {
      lastDebounceTime[n] = millis();
    }

    if ((millis() - lastDebounceTime[n]) > debounceDelay) {
      buttonState[n] = reading;
    }

    lastButtonState[n] = reading;
  }
}

// ===============================
// CALIBRATION
// ===============================
void handleCalibration() {

  if (!calibrating) return;

  // ---- CLOSE ----
  if (calStage == 0) {

    stepper->setSpeedInHz(CAL_SPEED);
    stepper->runBackward();

    if (digitalRead(closeStopLimit) == LOW) {

      stepper->forceStop();
      delay(100);

      stepper->move(HOMING_BACKOFF_STEPS);
      while (stepper->isRunning());

      delay(100);

      stepper->setSpeedInHz(HOMING_SLOW_SPEED);
      stepper->runBackward();

      while (digitalRead(closeStopLimit) == HIGH);

      stepper->forceStop();
      stepper->setCurrentPosition(0);

      calStage = 1;
      Serial.println("CLOSE HOME SET");
    }
  }

  // ---- OPEN ----
  else if (calStage == 1) {

    stepper->setSpeedInHz(CAL_SPEED);
    stepper->runForward();

    // 🔹 LIVE STEP PRINT
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 100) {
      lastPrint = millis();
      Serial.print("Cal Pos: ");
      Serial.println(stepper->getCurrentPosition());
    }

    if (digitalRead(openStopLimit) == LOW) {

      stepper->forceStop();

      fullTravelSteps = stepper->getCurrentPosition();

      Serial.println("=== CALIBRATION COMPLETE ===");
      Serial.print("Full travel steps: ");
      Serial.println(fullTravelSteps);
      Serial.println("============================");

      stepper->setSpeedInHz(maxSpeed);
      stepper->setAcceleration(accel);

      calibrating = false;
      motionArmed = true;
    }
  }
}

// ===============================
// BUTTONS
// ===============================
void handleButtons() {

  if (!motionArmed) return;

  if (buttonState[0] == LOW) {
    stepper->moveTo(fullTravelSteps);
    moving = true;

    digitalWrite(ledPin[1], HIGH);
    digitalWrite(ledPin[0], LOW);
  }

  if (buttonState[1] == LOW) {
    stepper->moveTo(0);
    moving = true;

    digitalWrite(ledPin[0], HIGH);
    digitalWrite(ledPin[1], LOW);
  }
}

// ===============================
// STATE
// ===============================
void updateMotionState() {

  if (moving && !stepper->isRunning()) {
    moving = false;

    digitalWrite(ledPin[0], LOW);
    digitalWrite(ledPin[1], LOW);
  }
}

// ===============================
// LOOP
// ===============================
void loop() {

  debounce();

  if (calibrating) {
    handleCalibration();
    return;
  }

  handleButtons();
  updateMotionState();
}

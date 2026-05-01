#include <AccelStepper.h>
#include <LiquidCrystal_I2C.h>

#define numLED 2
#define numBTN 2

//Motor Drive Pins
const int stepPin = 2;
const int dirPin  = 3;
const int enablePin = 4;

//manual drive pots
const int potPin1 = A1;
const int potPin2 = A2;
int potValue_L;
int potValue_R;

//Button and LED pins
//const int closeLED = 6;
//const int openLED  = 49;

const int openStopLimit  = 50;
const int closeStopLimit = 46;

// buttons / LEDs
int btnPin[numBTN] = {48,7};
int ledPin[numLED] = {6,49}; //closeLED: 6 - openLED: 49

//Debounce Variables
int buttonState[numBTN] = {HIGH, HIGH};
int lastButtonState[numBTN] = {HIGH, HIGH};

unsigned long lastDebounceTime[numBTN] = {0, 0};
const unsigned long debounceDelay = 50;

AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);
LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

//Speed and Accel. Values - ADJUSTABLE
float maxSpeed = 30000;
float accel    = 3000;
const float CAL_SPEED = 1500;

//CALIBRATION
const int HOMING_BACKOFF_STEPS = 300;
const float HOMING_SLOW_SPEED = 1200;

bool calibrating = true;
int calStage = 0;
bool motionArmed = false;
bool moving = false;
long fullTravelSteps = 0;


void setup() {

  Serial.begin(9600);

  lcd.init();                      // initialize the lcd 
  // Print a message to the LCD.
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Position: ");
  lcd.setCursor(0,1);
  lcd.print("Speed: ");

  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, LOW);

  for (int i = 0; i < numBTN; i++) {
    pinMode(btnPin[i], INPUT_PULLUP);
  }

  pinMode(openStopLimit, INPUT_PULLUP);
  pinMode(closeStopLimit, INPUT_PULLUP);

  for (int i=0; i < numLED; i++) {
    pinMode(ledPin[i], OUTPUT);
    digitalWrite(ledPin[i], LOW);
  }

  //cleaner step edges
  stepper.setMinPulseWidth(4);

  stepper.setMaxSpeed(maxSpeed);
  stepper.setAcceleration(accel);

  Serial.println("BOOT OK");
}


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



void handleButtons() {

  if (!motionArmed) return;

    if (buttonState[0] == LOW) {
      stepper.moveTo(fullTravelSteps);
      moving = true;

      digitalWrite(ledPin[1], HIGH);
      digitalWrite(ledPin[0], LOW);
    }

    if (buttonState[1] == LOW) {
      stepper.moveTo(0);
      moving = true;

      digitalWrite(ledPin[0], HIGH);
      digitalWrite(ledPin[1], LOW);
    }
}


void limitPins() {

  long pos = stepper.currentPosition();

  if ((digitalRead(openStopLimit) == LOW) && (stepper.targetPosition() > pos)) {
    stepper.setCurrentPosition(fullTravelSteps);
    stepper.stop();
  }

  if ((digitalRead(closeStopLimit) == LOW) && (stepper.targetPosition() < pos)) {
    stepper.setCurrentPosition(0);
    stepper.stop();
  }
}

void updateMotionState() {

  if (moving && stepper.distanceToGo() == 0) {

    moving = false;

    digitalWrite(ledPin[1], LOW);
    digitalWrite(ledPin[0], LOW);
  }
}

//eliminate idle buzz
void quietHold() {
  if (stepper.distanceToGo() == 0) {
    stepper.setSpeed(0);
  }
}


/*
void manualDrive()
{
  static long lastTarget = -999999;

  int pot = analogRead(potPin2);

  long targetPosition = map(pot, 0, 1023, fullTravelSteps, 0);

  const long POSITION_DEADBAND = 50; // steps

  if (abs(targetPosition - lastTarget) > POSITION_DEADBAND) {
    stepper.moveTo(targetPosition);
    lastTarget = targetPosition;
  }

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) {
    Serial.print("pot: ");
    Serial.print(pot);
    Serial.print(" target: ");
    Serial.print(targetPosition);
    Serial.print(" current: ");
    Serial.println(stepper.currentPosition());
    lastPrint = millis();
  }
}
*/

void loop() {

  debounce();

  if (calibrating) {
    handleCalibration();
    return;
  }

  handleButtons();
  limitPins();
  updateMotionState();
  quietHold();

  stepper.run();
}
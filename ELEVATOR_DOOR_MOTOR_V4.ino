/* Elevator door */
#define numLED 2
#define numBTN 2
#define QUEUE_SIZE 5

// motor pins
const int enablePinL = 4;
const int enablePinR = 5;
const int LPWMPin = 3;
const int RPWMPin = 2;

// buttons / LEDs
int btnPin[numBTN] = {7,48};
int ledPin[numLED] = {6,49};

// pots
const int potPin1 = A1;
const int potPin2 = A2;

// limit switches
const int limitPin_L = 50;
const int limitPin_R = 51;
const int limitPin_close = 46;
const int limitPin_open = 47;

// ===============================
// MOTION PROFILES
// ===============================
const int leftRampUpSteps = 50;
const int leftRampDownSteps = 130;
const int leftHoldPWM = 90;

const int rightRampUpSteps = 50;
const int rightRampDownSteps = 140;
const int rightHoldPWM = 45;

const int maxPWM = 230;

// ===============================
// QUEUE SYSTEM
// ===============================
enum Command {
  CMD_NONE,
  CMD_LEFT,
  CMD_RIGHT
};

Command commandQueue[QUEUE_SIZE];
int queueHead = 0;
int queueTail = 0;
int queueCount = 0;

// ===============================
// BUTTON SYSTEM
// ===============================
int buttonState[numBTN] = {HIGH, HIGH};
int lastButtonState[numBTN] = {HIGH, HIGH};
bool buttonPressed[numBTN] = {false, false};

unsigned long lastDebounceTime[numBTN] = {0, 0};
const unsigned long debounceDelay = 50;

// ===============================
// MOTION CONTROL FLAGS
// ===============================
bool motionActive = false;

// ===============================
// LED STATE (LATCHED)
// ===============================
bool ledActive[numBTN] = {false, false};

// ===============================
// MOTOR STATE MACHINE
// ===============================
enum MotorState {
  OFF,
  RAMPUP,
  TRAVEL_HOLD,
  RAMPDOWN,
  DWELL_HOLD,
  FINAL_RAMPDOWN
};

enum MotorDirection {
  NONE,
  LEFT_DIR,
  RIGHT_DIR
};

MotorState state = OFF;
MotorDirection direction = NONE;

int currentPWM = 0;
int rampCounter = 0;

// ===============================
// QUEUE FUNCTIONS
// ===============================
bool enqueue(Command cmd) {

  if (queueCount >= QUEUE_SIZE) return false;

  commandQueue[queueTail] = cmd;
  queueTail = (queueTail + 1) % QUEUE_SIZE;
  queueCount++;

  return true;
}

Command dequeue() {

  if (queueCount == 0) return CMD_NONE;

  Command cmd = commandQueue[queueHead];
  queueHead = (queueHead + 1) % QUEUE_SIZE;
  queueCount--;

  return cmd;
}

// ===============================
// SETUP
// ===============================
void setup() {

  Serial.begin(9600);

  pinMode(enablePinL, OUTPUT);
  pinMode(enablePinR, OUTPUT);
  pinMode(LPWMPin, OUTPUT);
  pinMode(RPWMPin, OUTPUT);

  pinMode(limitPin_L, INPUT_PULLUP);
  pinMode(limitPin_R, INPUT_PULLUP);
  pinMode(limitPin_close, INPUT_PULLUP);
  pinMode(limitPin_open, INPUT_PULLUP);

  digitalWrite(enablePinL, HIGH);
  digitalWrite(enablePinR, HIGH);

  for (int i = 0; i < numBTN; i++) {
    pinMode(btnPin[i], INPUT_PULLUP);
  }

  for (int i = 0; i < numLED; i++) {
    pinMode(ledPin[i], OUTPUT);
    digitalWrite(ledPin[i], LOW);
  }
}

// ===============================
// DEBOUNCE (STABLE EDGE)
// ===============================
void debounce() {

  for (int n = 0; n < numBTN; n++) {

    int reading = digitalRead(btnPin[n]);

    if (reading != lastButtonState[n]) {
      lastDebounceTime[n] = millis();
    }

    if ((millis() - lastDebounceTime[n]) > debounceDelay) {

      if (reading == LOW && buttonState[n] == HIGH) {
        buttonPressed[n] = true;
      }

      buttonState[n] = reading;
    }

    lastButtonState[n] = reading;
  }
}

// ===============================
// STOP MOTION (SINGLE SOURCE OF TRUTH)
// ===============================
void stopMotion() {

  currentPWM = 0;
  state = OFF;
  direction = NONE;
  motionActive = false;

  ledActive[0] = false;
  ledActive[1] = false;
}

// ===============================
void startMotion(MotorDirection dir) {

  direction = dir;
  state = RAMPUP;
  rampCounter = 0;
  motionActive = true;

  if (dir == LEFT_DIR) ledActive[0] = true;
  if (dir == RIGHT_DIR) ledActive[1] = true;
}

// ===============================
// LED CONTROL (NO FLICKER)
// ===============================
void updateLEDs() {

  if (!motionActive) {
    ledActive[0] = false;
    ledActive[1] = false;
  }

  digitalWrite(ledPin[0], ledActive[0]);
  digitalWrite(ledPin[1], ledActive[1]);
}

// ===============================
void driveManualMotor() {

  int pwmL = map(analogRead(potPin1), 0, 1023, 0, 255);
  int pwmR = map(analogRead(potPin2), 0, 1023, 0, 255);

  analogWrite(LPWMPin, pwmL);
  analogWrite(RPWMPin, pwmR);
}

// ===============================
void driveAutoMotor() {

  if (direction == LEFT_DIR) {
    analogWrite(LPWMPin, currentPWM);
    analogWrite(RPWMPin, 0);
  } 
  else if (direction == RIGHT_DIR) {
    analogWrite(LPWMPin, 0);
    analogWrite(RPWMPin, currentPWM);
  } 
  else {
    analogWrite(LPWMPin, 0);
    analogWrite(RPWMPin, 0);
  }
}

// ===============================
void updateMotorState() {

  int Limit_L = digitalRead(limitPin_L);
  int Limit_R = digitalRead(limitPin_R);

  int limitClose = digitalRead(limitPin_close);
  int limitOpen  = digitalRead(limitPin_open);

  int rampUpSteps = (direction == LEFT_DIR) ? leftRampUpSteps : rightRampUpSteps;
  int rampDownSteps = (direction == LEFT_DIR) ? leftRampDownSteps : rightRampDownSteps;
  int holdPWM = (direction == LEFT_DIR) ? leftHoldPWM : rightHoldPWM;

  switch (state) {

    case OFF:
      currentPWM = 0;
      break;

    case RAMPUP:

      if (rampCounter < rampUpSteps) {
        currentPWM = map(rampCounter, 0, rampUpSteps, 0, maxPWM);
        rampCounter++;
      } else {
        currentPWM = maxPWM;
        state = TRAVEL_HOLD;
      }
      break;

    case TRAVEL_HOLD:

      currentPWM = maxPWM;

      if ((direction == LEFT_DIR && Limit_L == LOW) ||
          (direction == RIGHT_DIR && Limit_R == LOW)) {

        state = RAMPDOWN;
        rampCounter = 0;
      }

      if (limitOpen == LOW || limitClose == LOW) {
        stopMotion();
        return;
      }

      break;

    case RAMPDOWN:

      if (rampCounter < rampDownSteps) {

        currentPWM = map(rampCounter, 0, rampDownSteps, maxPWM, holdPWM);
        rampCounter++;

      } else {
        currentPWM = holdPWM;
        state = DWELL_HOLD;
      }
      break;

    case DWELL_HOLD:

      currentPWM = holdPWM;

      if (limitOpen == LOW || limitClose == LOW) {
        rampCounter = 0;
        state = FINAL_RAMPDOWN;
      }

      break;

    case FINAL_RAMPDOWN:

      if (rampCounter < 50) {
        currentPWM = map(rampCounter, 0, 50, holdPWM, 0);
        rampCounter++;
      } else {
        stopMotion();
      }

      break;
  }
}

// ===============================
void processQueue() {

  if (motionActive) return;

  Command cmd = dequeue();

  if (cmd == CMD_LEFT) startMotion(LEFT_DIR);
  else if (cmd == CMD_RIGHT) startMotion(RIGHT_DIR);
}

// ===============================
// LOOP
// ===============================
void loop() {

  debounce();
  updateLEDs();

  for (int n = 0; n < numBTN; n++) {

    if (buttonPressed[n]) {

      if (n == 0) enqueue(CMD_LEFT);
      else if (n == 1) enqueue(CMD_RIGHT);

      buttonPressed[n] = false;
    }
  }

  processQueue();

  if (state == OFF) {
    driveManualMotor();
  } else {
    updateMotorState();
    driveAutoMotor();
  }

  delay(10);
}
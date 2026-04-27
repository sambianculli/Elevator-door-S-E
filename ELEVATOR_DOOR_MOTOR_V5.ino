/* Elevator door - FINAL CLEAN LIMIT HANDLING */

#define numLED 2
#define numBTN 2

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

// limits
const int limitPin_L = 50;
const int limitPin_R = 51;
const int limitPin_close = 46;
const int limitPin_open = 47;

// ===============================
const int maxPWM_LEFT = 230;
const int maxPWM_RIGHT = 210;

const int leftRampUpSteps = 50;
const int rightRampUpSteps = 50;

const int leftRampDownSteps = 145;
const int rightRampDownSteps = 195;

const int leftHoldPWM = 100;
const int rightHoldPWM = 100;

const int openRampSteps = 110;
const int closeRampSteps = 43;
const int timeoutRampSteps = 50;

// ===============================
const unsigned long motionTimeout = 7000;
unsigned long motionStartTime = 0;
bool timeoutActive = false;

// ===============================
enum StopReason {
  STOP_NONE,
  STOP_OPEN,
  STOP_CLOSE,
  STOP_TIMEOUT
};

StopReason stopReason = STOP_NONE;

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

bool motionActive = false;

int currentPWM = 0;
int rampCounter = 0;

// ===============================
// BUTTONS
// ===============================
int buttonState[numBTN] = {HIGH, HIGH};
int lastButtonState[numBTN] = {HIGH, HIGH};

unsigned long lastDebounceTime[numBTN] = {0, 0};
const unsigned long debounceDelay = 50;

// ===============================
bool ledActive[numBTN] = {false, false};

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
  }
}

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
bool buttonPressedEdge(int n) {
  static int lastStable[numBTN] = {HIGH, HIGH};

  bool pressed = false;

  if (buttonState[n] == LOW && lastStable[n] == HIGH) {
    pressed = true;
  }

  lastStable[n] = buttonState[n];
  return pressed;
}

// ===============================
void startMotion(MotorDirection dir) {

  direction = dir;
  state = RAMPUP;
  rampCounter = 0;

  motionActive = true;
  timeoutActive = false;
  motionStartTime = millis();

  stopReason = STOP_NONE;

  ledActive[0] = false;
  ledActive[1] = false;

  if (dir == LEFT_DIR) ledActive[0] = true;
  if (dir == RIGHT_DIR) ledActive[1] = true;
}

// ===============================
void stopMotion() {

  currentPWM = 0;
  state = OFF;
  direction = NONE;

  motionActive = false;
  timeoutActive = false;

  ledActive[0] = false;
  ledActive[1] = false;
}

// ===============================
void checkTimeout() {

  if (!motionActive || timeoutActive) return;

  if (millis() - motionStartTime > motionTimeout) {
    timeoutActive = true;
    stopReason = STOP_TIMEOUT;
    state = FINAL_RAMPDOWN;
    rampCounter = 0;
  }
}

// ===============================
void updateLEDs() {
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
  } else {
    analogWrite(LPWMPin, 0);
    analogWrite(RPWMPin, currentPWM);
  }
}

// ===============================
void updateMotorState() {

  int Limit_L = digitalRead(limitPin_L);
  int Limit_R = digitalRead(limitPin_R);
  int limitOpen = digitalRead(limitPin_open);
  int limitClose = digitalRead(limitPin_close);

  int maxPWM = (direction == LEFT_DIR) ? maxPWM_LEFT : maxPWM_RIGHT;
  int rampUp = (direction == LEFT_DIR) ? leftRampUpSteps : rightRampUpSteps;
  int rampDown = (direction == LEFT_DIR) ? leftRampDownSteps : rightRampDownSteps;
  int holdPWM = (direction == LEFT_DIR) ? leftHoldPWM : rightHoldPWM;

  // 🔥 GLOBAL LIMIT OVERRIDE (runs every loop)
  if (motionActive && state != FINAL_RAMPDOWN && state != OFF) {

    if (direction == RIGHT_DIR && limitOpen == LOW) {
      stopReason = STOP_OPEN;
      state = FINAL_RAMPDOWN;
      rampCounter = 0;
    }

    if (direction == LEFT_DIR && limitClose == LOW) {
      stopReason = STOP_CLOSE;
      state = FINAL_RAMPDOWN;
      rampCounter = 0;
    }
  }

  switch (state) {

    case OFF:
      currentPWM = 0;
      break;

    case RAMPUP:
      if (rampCounter < rampUp) {
        currentPWM = map(rampCounter++, 0, rampUp, 0, maxPWM);
      } else {
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

      break;

    case RAMPDOWN:
      if (rampCounter < rampDown) {
        currentPWM = map(rampCounter++, 0, rampDown, maxPWM, holdPWM);
      } else {
        state = DWELL_HOLD;
      }
      break;

    case DWELL_HOLD:
      currentPWM = holdPWM;
      break;

    case FINAL_RAMPDOWN: {
      int steps = (stopReason == STOP_CLOSE) ? closeRampSteps :
                  (stopReason == STOP_OPEN) ? openRampSteps :
                  timeoutRampSteps;

      if (rampCounter < steps) {
        currentPWM = map(rampCounter++, 0, steps, holdPWM, 0);
      } else {
        stopMotion();
      }
      break;
    }
  }
}

// ===============================
void loop() {

  debounce();
  checkTimeout();
  updateLEDs();

  if (!motionActive) {

    if (buttonPressedEdge(0)) {
      startMotion(LEFT_DIR);
    }

    if (buttonPressedEdge(1)) {
      startMotion(RIGHT_DIR);
    }
  }

  if (!motionActive) {
    driveManualMotor();
  } else {
    updateMotorState();
    driveAutoMotor();
  }

  delay(10);
}
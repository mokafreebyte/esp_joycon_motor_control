#include <Bluepad32.h>

// ================= MOTOR STATE/CONFIG =================
hw_timer_t* stepTimer = nullptr;
volatile uint32_t stepPeriodUs = 0;
volatile bool doFineStep = false;

volatile bool motor1Enabled = false;
volatile bool motor1Dir = true;        
volatile bool motor1stepState = false;

volatile bool motor2Enabled = false;
volatile bool motor2Dir = true;        
volatile bool motor2stepState = false;

constexpr bool MOTOR1DIRECTION = true;            // Invert to change Motor 1 control Direction
constexpr bool MOTOR2DIRECTION = true;            // Invert to change Motor 2 control Direction
constexpr float NORMAL_RPS = 0.5;            // Change Motor speeds here
constexpr float FINE_RPS = 0.05;

static bool lastMotor1State = false;
static bool lastMotor2State = false;

#define STEPS_PER_REV 1600
#define TIMER_PRESCALER 80          // 80 MHz / 80 = 1 MHz → 1 tick = 1 µs

#define STEP_PIN_1 23
#define STEP_PIN_2 21
#define DIR_PIN_1 22
#define DIR_PIN_2 19
#define EN_PIN 0

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// Joy-Con calibration values (based on your actual idle readings)
const int JOYCON_IDLE_X = -55;
const int JOYCON_IDLE_Y = 13;
const int DEADZONE_THRESHOLD = 25;

// Joy-Con movement ranges (based on your actual readings)
const int JOYCON_LEFT_THRESHOLD = -200;   // Left: -417
const int JOYCON_RIGHT_THRESHOLD = 150;   // Right: 305
const int JOYCON_UP_THRESHOLD = -200;     // Up: -440
const int JOYCON_DOWN_THRESHOLD = 200;    // Down: 440



// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      // Additionally, you can get certain gamepad properties like:
      // Model, VID, PID, BTAddr, flags, etc.
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id, properties.product_id);
      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
      }
    }

    if (!foundEmptySlot) {
      Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;
  motor1Enabled = false;
  motor2Enabled = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }

    if (!foundController) {
      Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

// ========= MOTOR PULSE GENERATOR ========= //

void IRAM_ATTR onStepTimer() {
  if (motor1Enabled){
    digitalWrite(STEP_PIN_1, motor1stepState);
    motor1stepState = !motor1stepState;
  }
  if (motor2Enabled){
    digitalWrite(STEP_PIN_2, motor2stepState);
    motor2stepState = !motor2stepState;
  }
}

void updateStepPeriod() {
  float rps = doFineStep ? FINE_RPS : NORMAL_RPS;

  stepPeriodUs = 1000000.0 / (rps * STEPS_PER_REV * 2);
  timerAlarmWrite(stepTimer, stepPeriodUs, true);
}

// ========= GAME CONTROLLER ACTIONS SECTION ========= //

void processGamepad(ControllerPtr ctl) {
  // There are different ways to query whether a button is pressed.
  // By query each button individually:
  //  a(), b(), x(), y(), l1(), etc...

  bool move1ThisIteration = false;
  bool move2ThisIteration = false;
 
  //== Joy-Con A button = 0x0001 ==//
  static bool prevAButton = false;
  bool currentAButton = (ctl->buttons() & 0x0001);
  if (currentAButton && !prevAButton) {
    Serial.println("Button: A PRESSED");
    // code for when A button is pushed
  }
  if (!currentAButton && prevAButton) {
    Serial.println("Button: A RELEASED");
    // code for when A button is released
  }
  prevAButton = currentAButton;

  //== Joy-Con X button = 0x0002 ==//
  static bool prevXButton = false;
  bool currentXButton = (ctl->buttons() & 0x0002);
  if (currentXButton && !prevXButton) {
    Serial.println("Button: X PRESSED");
    doFineStep = true;
    updateStepPeriod();
  }
  if (!currentXButton && prevXButton) {
    Serial.println("Button: X RELEASED");
    doFineStep = false;
    updateStepPeriod();
  }
  prevXButton = currentXButton;

  //== Joy-Con B button = 0x0004 ==//
  static bool prevBButton = false;
  bool currentBButton = (ctl->buttons() & 0x0004);
  if (currentBButton && !prevBButton) {
    Serial.println("Button: B PRESSED");
    // code for when B button is pushed
  }
  if (!currentBButton && prevBButton) {
    Serial.println("Button: B RELEASED");
    // code for when B button is released
  }
  prevBButton = currentBButton;

  //== Joy-Con Y button = 0x0008 ==//
  static bool prevYButton = false;
  bool currentYButton = (ctl->buttons() & 0x0008);
  if (currentYButton && !prevYButton) {
    Serial.println("Button: Y PRESSED");
    // code for when Y button is pushed
  }
  if (!currentYButton && prevYButton) {
    Serial.println("Button: Y RELEASED");
    // code for when Y button is released
  }
  prevYButton = currentYButton;

  // Calculate calibrated axis values
  int calibratedX = ctl->axisX() - JOYCON_IDLE_X;
  int calibratedY = ctl->axisY() - JOYCON_IDLE_Y;

  //== LEFT JOYSTICK - UP ==//
  // Based on your actual reading: Up: axis L: -50, -440
  if (ctl->axisY() <= JOYCON_UP_THRESHOLD) {
    Serial.println("Motor 1: UP");
    motor1Dir = MOTOR1DIRECTION;
    move1ThisIteration = true;
  }

  //== LEFT JOYSTICK - DOWN ==//
  // Based on your actual reading: Down: axis L: -100, 440
  else if (ctl->axisY() >= JOYCON_DOWN_THRESHOLD) {
    Serial.println("Motor 1: DOWN");
    motor1Dir = !MOTOR1DIRECTION;    
    move1ThisIteration = true;
  }

  //== LEFT JOYSTICK - LEFT ==//
  // Based on your actual reading: Left: axis L: -417, -30
  if (ctl->axisX() <= JOYCON_LEFT_THRESHOLD) {
    Serial.println("Motor 2: UP");
    motor2Dir = MOTOR2DIRECTION;
    move2ThisIteration = true;
  }

  //== LEFT JOYSTICK - RIGHT ==//
  // Based on your actual reading: Right: axis L: 305, -29
  if (ctl->axisX() >= JOYCON_RIGHT_THRESHOLD) {
    Serial.println("Motor 2: DOWN");
    motor2Dir = !MOTOR2DIRECTION;
    move2ThisIteration = true;
  }

  //== LEFT JOYSTICK DEADZONE (Joy-Con calibrated) ==//
  // Based on your neutral reading: axis L: -55, 13
  if (abs(calibratedX) < DEADZONE_THRESHOLD && abs(calibratedY) < DEADZONE_THRESHOLD) {
    //move1ThisIteration = false;
  }

  if (move1ThisIteration != lastMotor1State) {
    motor1Enabled = move1ThisIteration;
    lastMotor1State = move1ThisIteration;
  }
  if (move2ThisIteration != lastMotor2State) {
    motor2Enabled = move2ThisIteration;
    lastMotor2State = move2ThisIteration;
  }
  digitalWrite(DIR_PIN_1, motor1Dir);
  digitalWrite(DIR_PIN_2, motor2Dir);
}

void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
         processGamepad(myController);
      }
      else {
        Serial.println("Unsupported controller");
      }
    }
  }
}

// Arduino setup function. Runs in CPU 1
void setup() {
  pinMode(STEP_PIN_1, OUTPUT);
  pinMode(DIR_PIN_1, OUTPUT);
  pinMode(STEP_PIN_2, OUTPUT);
  pinMode(DIR_PIN_2, OUTPUT);
  pinMode(EN_PIN, OUTPUT);

  digitalWrite(STEP_PIN_1, HIGH);
  digitalWrite(DIR_PIN_1, HIGH);
  digitalWrite(STEP_PIN_2, HIGH);
  digitalWrite(DIR_PIN_2, HIGH);
  digitalWrite(EN_PIN, LOW);   // enable TB6600

  Serial.begin(115200);
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);

  BP32.forgetBluetoothKeys();

  BP32.enableVirtualDevice(false);

  // ====== STEP TIMER SETUP ======
  stepTimer = timerBegin(0, TIMER_PRESCALER, true);
  timerAttachInterrupt(stepTimer, &onStepTimer, true);

  updateStepPeriod();       
  timerAlarmEnable(stepTimer);

}

// Arduino loop function. Runs in CPU 1.
void loop() {
  // This call fetches all the controllers' data.
  // Call this function in your main loop.
  bool dataUpdated = BP32.update();
  if (dataUpdated)
    processControllers();

    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise, the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

    vTaskDelay(1);
}
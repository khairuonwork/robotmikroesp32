

#include <Bluepad32.h>

#include <ESP32Servo.h>

// Pin Definitions
#define IN1 19  // Motor 1 forward
#define IN2 18  // Motor 1 backward
#define IN3 22  // Motor 2 forward
#define IN4 23  // Motor 2 backward
#define enA 25  // Speed control for Motor 1
#define enB 26  // Speed control for Motor 2

//Servo
#define SERVO_PIN_KIRI 13  // Servo control pin kiri

// Speed Constants
const int DEFAULT_SPEED = 125;
const int TURN_SPEED = 70;
const int TURN_SPEED_ENC = 50;
const int STOP_SPEED = 0;

// Servo Declaration
Servo servokiri;
Servo servokanan;

// Bluepad32 Controllers
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// Function Declarations
void moveForward();
void moveBackward();
void turnRight();
void turnLeft();
void stopMotors();
void setMotorSpeed(int speedA, int speedB);
void servomechanism(ControllerPtr ctl);
void processDpadInput(ControllerPtr ctl);
void processControllers();
void onConnectedController(ControllerPtr ctl);
void onDisconnectedController(ControllerPtr ctl);

// Motor Control Functions
void setMotorSpeed(int speedA, int speedB) {
    analogWrite(enA, speedA);
    analogWrite(enB, speedB);
}

void moveForward() {
    Serial.println("Moving Forward");
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    setMotorSpeed(DEFAULT_SPEED, DEFAULT_SPEED);
}

void moveBackward() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    setMotorSpeed(DEFAULT_SPEED, DEFAULT_SPEED);
}

void turnRight() {

    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    setMotorSpeed(DEFAULT_SPEED, TURN_SPEED);
}
void turnRightVector(){
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  setMotorSpeed(DEFAULT_SPEED, TURN_SPEED_ENC);
}

void turnLeft() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    setMotorSpeed(TURN_SPEED, DEFAULT_SPEED);
}
void turnLeftVector(){
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  setMotorSpeed(TURN_SPEED_ENC, DEFAULT_SPEED);
}

void stopMotors() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    setMotorSpeed(STOP_SPEED, STOP_SPEED);
}

// Servo Mechanism Control
void servomechanism(ControllerPtr ctl) {
    static unsigned long lastButtonPress = 0;
    const unsigned long debounceDelay = 200; // Debounce delay in ms
    uint16_t buttons = ctl->buttons();
    unsigned long currentTime = millis();

    if (currentTime - lastButtonPress > debounceDelay) {
        if (buttons & BUTTON_X) {
            Serial.println("Servo Turning to 90°");
            servokiri.write(90);
        } else if (buttons & BUTTON_A) {
            Serial.println("Servo Turning to 0°");
            servokiri.write(0);
        }
        lastButtonPress = currentTime;
    }
}

// DPAD Movement Processing
void processDpadInput(ControllerPtr ctl) {
    uint8_t dpad = ctl->dpad();

    if (dpad & DPAD_UP) {
        moveForward();
    }  else if (dpad & DPAD_DOWN) {
        moveBackward();
    } else if (dpad & DPAD_RIGHT) {
        turnRight();
    } else if (dpad & DPAD_LEFT) {
        turnLeft();
    } else {
        stopMotors();
    }
}

// Controller Event Handlers
void onConnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("\nController connected: Index %d\n", i);
            myControllers[i] = ctl;
            return;
        }
    }
    Serial.println("No available slot for the connected controller.");
}

void onDisconnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("\nController disconnected: Index %d\n", i);
            myControllers[i] = nullptr;
            stopMotors();
            return;
        }
    }
    Serial.println("Disconnected controller not found in the slot list.");
}

// Process All Connected Controllers
void processControllers() {
    for (auto ctl : myControllers) {
        if (ctl && ctl->isConnected() && ctl->hasData()) {
            processDpadInput(ctl);
            servomechanism(ctl);
        }
    }
}

// Setup Function
void setup() {
    Serial.begin(115200);
    Serial.println("Starting Bluepad32...");

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(enA, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(SERVO_PIN_KANAN, OUTPUT);
    pinMode(SERVO_PIN_KIRI, OUTPUT);

    servokiri.attach(SERVO_PIN_KIRI); // Attach the servo kiri
    servokiri.write(0);

    stopMotors(); // Stop motors during setup

    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys(); // Optional: Forget previously paired devices
}

// Main Loop
void loop() {
    BP32.update();
    processControllers();

    delay(10); // Small delay for responsiveness
}
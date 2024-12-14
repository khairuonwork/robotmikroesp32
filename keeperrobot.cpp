#include <Bluepad32.h>
#include <Servo.h>

#define IN1 18  // Motor 1 forward
#define IN2 19  // Motor 1 backward
#define IN3 22  // Motor 2 forward
#define IN4 23  // Motor 2 backward
#define enA 25  // Speed control for Motor 1
#define enB 26  // Speed control for Motor 2

const int DEFAULT_SPEED = 200;
const int STOP_SPEED = 0;

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

void right();
void left();
void stopMotors();
void taunting();
void setMotorSpeed(int speedA, int speedB);
void processDpadInput(ControllerPtr ctl);
void processControllers();
void onConnectedController(ControllerPtr ctl);
void onDisconnectedController(ControllerPtr ctl);

void setMotorSpeed(int speedA, int speedB){
  analogWrite(enA, speedA);
  analogWrite(enB, speedB);
}

void right(){
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  setMotorSpeed(DEFAULT_SPEED, DEFAULT_SPEED);
}

void left(){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  setMotorSpeed(DEFAULT_SPEED, DEFAULT_SPEED);
}

void taunting(ControllerPtr ctl){
  static unsigned long lastButtonPress = 0;
  const unsigned long debounceDelay = 200;
  uint16_t buttons = ctl->buttons();
  unsigned long currentTime = millis();

  if(currentTime - lastButtonPress > debounceDelay){
    if(buttons & BUTTON_CIRCLE){
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      setMotorSpeed(DEFAULT_SPEED, DEFAULT_SPEED);
    } else if (buttons & BUTTON_TRIANGLE){
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      setMotorSpeed(DEFAULT_SPEED, DEFAULT_SPEED);
    }
    lastButtonPress = currentTime;
  }
}

void stopMotors() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    setMotorSpeed(STOP_SPEED, STOP_SPEED);
}

void processDpadInput(ControllerPtr ctl){
  uint8_t dpad = ctl->dpad();
  
  if (dpad & DPAD_RIGHT){
    right();
  } else if (dpad & DPAD_LEFT){
    left();
  } else {
    stop();
  }
}

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

void processControllers() {
    for (auto ctl : myControllers) {
        if (ctl && ctl->isConnected() && ctl->hasData()) {
            processDpadInput(ctl);
            taunting(ctl);
        }
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("Starting Bluepad32...");

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(enA, OUTPUT);
    pinMode(enB, OUTPUT);

    stop();
     BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys(); 
}

void loop() {
     if (BP32.update()) {
        processControllers();
    }
    delay(10); 

}

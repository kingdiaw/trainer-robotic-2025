#include <TwoWDController.h>

#define TIMER_INTERVAL 33

int leftMotor[] = { 5, 25, 2 };
int rightMotor[] = { 14, 23, 27 };
int leftEnc[] = { 18, 19 };
int rightEnc[] = { 16, 17 };
long lastMillis;
int debugTick;

TwoWDController robot(leftMotor, rightMotor, leftEnc, rightEnc);

void setup() {
  Serial.begin(115200);
  robot.begin();
  robot.drive(30, 30);  // Set initial speed
}

void loop() {
  if (millis() - lastMillis >= TIMER_INTERVAL) {
    lastMillis = millis();
    robot.updatePID();
  }

  if (millils() > debugTick) {
    debugTick = millis() + 500;

    Serial.print("Left Encoder: ");
    Serial.print(robot.readEncoder(0));
    Serial.print(" Right Encoder: ");
    Serial.println(robot.readEncoder(1));
  }
}

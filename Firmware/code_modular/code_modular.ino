#include <Arduino.h>
#include "PCF8574.h"
#include "encoder.h"
#include "DriverL298.h"
#include "PIDControl.h"
#include <Ticker.h>

// ESP32 GPIO Mode
// DriverL298 motor(2, 4, 5, 15, 18, 19);

// PCF8574 I2C Mode
DriverL298 motorDriver(0x20, 2, 15, P0, P1, P2, P3);
PIDControl pidController;
Ticker repeat;

// Static wrapper function
void IRAM_ATTR updatePIDWrapper() {
  pidController.updatePID();
}

void setup() {
  Serial.begin(115200);

  // Configure encoder pins
  pinMode(LEFT_ENC_PIN_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_PIN_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_PIN_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_PIN_B, INPUT_PULLUP);

  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_B), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN_A), rightEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN_B), rightEncoderISR, CHANGE);

  repeat.attach(0.033, updatePIDWrapper);
  pidController.resetPID();
  motorDriver.begin();

  //Set target speed
  pidController.drive(0.12, 0);
}

long debugTick;
long motorTick;
void loop() {

  if (millis() > debugTick) {
    debugTick = millis() + 500;
    // Debugging: Print encoder values and speeds
    long leftTicks = readEncoder(0);
    // long rightTicks = readEncoder(1); // (commented out)

    double leftSpeed = pidController.getSpeedMetersPerSecond(pidController.getPrevInput(0));

    Serial.print("Left Speed (m/s): ");
    Serial.print(leftSpeed);
    Serial.print(" | PWM: ");
    Serial.print(pidController.getOutput(0));
    Serial.print(" | TargetTPF: ");
    Serial.print(pidController.getTargetTicksPerFrame(0));
    Serial.print(" | ActualTPF: ");
    Serial.print(pidController.getPrevInput(0));
    Serial.print(" | Revs/min: ");
    Serial.print(pidController.getSpeedRevPerMinute(0));

    Serial.println();
  }
}

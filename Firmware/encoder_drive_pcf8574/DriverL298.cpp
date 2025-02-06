#include "DriverL298.h"

// Constructor for ESP32 GPIO control
DriverL298::DriverL298(int leftEnable, int leftIn1, int leftIn2, int rightEnable, int rightIn1, int rightIn2)
    : leftEnablePin(leftEnable), leftIn1Pin(leftIn1), leftIn2Pin(leftIn2),
      rightEnablePin(rightEnable), rightIn1Pin(rightIn1), rightIn2Pin(rightIn2),
      mode(ESP32_GPIO), pcf8574(nullptr) {}

// Constructor for PCF8574 control
DriverL298::DriverL298(int i2cAddr, int leftEnable, int rightEnable, int leftIn1, int leftIn2, int rightIn1, int rightIn2)
    : i2cAddress(i2cAddr), leftEnablePin(leftEnable), rightEnablePin(rightEnable),
      leftIn1Pin(leftIn1), leftIn2Pin(leftIn2), rightIn1Pin(rightIn1), rightIn2Pin(rightIn2),
      mode(PCF8574_IO) {
    pcf8574 = new PCF8574(i2cAddr);
}

void DriverL298::begin() {
    if (mode == ESP32_GPIO) {
        pinMode(leftEnablePin, OUTPUT);
        pinMode(leftIn1Pin, OUTPUT);
        pinMode(leftIn2Pin, OUTPUT);
        pinMode(rightEnablePin, OUTPUT);
        pinMode(rightIn1Pin, OUTPUT);
        pinMode(rightIn2Pin, OUTPUT);
    } else {
        pcf8574->pinMode(leftIn1Pin, OUTPUT);
        pcf8574->pinMode(leftIn2Pin, OUTPUT);
        pcf8574->pinMode(rightIn1Pin, OUTPUT);
        pcf8574->pinMode(rightIn2Pin, OUTPUT);
        pcf8574->begin();
    }
}

void DriverL298::setMotorSpeeds(int leftSpeed, int rightSpeed) {
    if (mode == ESP32_GPIO) {
        // Left motor control
        digitalWrite(leftIn1Pin, leftSpeed >= 0 ? HIGH : LOW);
        digitalWrite(leftIn2Pin, leftSpeed >= 0 ? LOW : HIGH);
        analogWrite(leftEnablePin, constrain(abs(leftSpeed), 0, 255));

        // Right motor control
        digitalWrite(rightIn1Pin, rightSpeed >= 0 ? HIGH : LOW);
        digitalWrite(rightIn2Pin, rightSpeed >= 0 ? LOW : HIGH);
        analogWrite(rightEnablePin, constrain(abs(rightSpeed), 0, 255));

    } else {
        // Left motor control
        pcf8574->digitalWrite(leftIn1Pin, leftSpeed >= 0 ? HIGH : LOW);
        pcf8574->digitalWrite(leftIn2Pin, leftSpeed >= 0 ? LOW : HIGH);
        analogWrite(leftEnablePin, constrain(abs(leftSpeed), 0, 255));

        // Right motor control
        pcf8574->digitalWrite(rightIn1Pin, rightSpeed >= 0 ? HIGH : LOW);
        pcf8574->digitalWrite(rightIn2Pin, rightSpeed >= 0 ? LOW : HIGH);
        analogWrite(rightEnablePin, constrain(abs(rightSpeed), 0, 255));
    }
}

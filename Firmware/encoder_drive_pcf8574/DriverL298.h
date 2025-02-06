#ifndef DRIVER_L298_H
#define DRIVER_L298_H

#include <Arduino.h>
#include "PCF8574.h"

class DriverL298 {
public:
    enum ControlMode { ESP32_GPIO, PCF8574_IO };

    // Constructor for ESP32 GPIO control
    DriverL298(int leftEnable, int leftIn1, int leftIn2, int rightEnable, int rightIn1, int rightIn2);

    // Constructor for PCF8574 control
    DriverL298(int i2cAddress, int leftEnable, int rightEnable, int leftIn1, int leftIn2, int rightIn1, int rightIn2);

    void begin();
    void setMotorSpeeds(int leftSpeed, int rightSpeed);

private:
    ControlMode mode;
    int leftEnablePin, leftIn1Pin, leftIn2Pin;
    int rightEnablePin, rightIn1Pin, rightIn2Pin;
    int i2cAddress;
    PCF8574 *pcf8574;
};

#endif

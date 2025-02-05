#ifndef TwoWDController_h
#define TwoWDController_h

#include <Arduino.h>

class TwoWDController {
public:
    TwoWDController(int leftMotorPins[], int rightMotorPins[], int leftEncPins[], int rightEncPins[]);
    void begin();
    void drive(double leftSpeed, double rightSpeed);
    void updatePID();
    long readEncoder(int i);
    void resetEncoders();
    
private:
    int leftMotorPins[3];
    int rightMotorPins[3];
    int leftEncPins[2];
    int rightEncPins[2];
    
    volatile long leftEncPos = 0;
    volatile long rightEncPos = 0;

    struct PID {
        double TargetTicksPerFrame;
        long Encoder;
        long PrevEnc;
        int PrevInput;
        int ITerm;
        long output;
    };
    
    PID leftPID, rightPID;

    void doPID(PID* p);
    void setMotorSpeeds(int leftSpeed, int rightSpeed);
    void resetPID();
    
    static void IRAM_ATTR leftEncoderISR();
    static void IRAM_ATTR rightEncoderISR();
};

#endif

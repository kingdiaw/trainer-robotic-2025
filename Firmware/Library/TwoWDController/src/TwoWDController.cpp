#include "TwoWDController.h"

static const int8_t ENC_STATES[] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};

// Global variables for encoder counts
volatile long leftEncPos = 0;
volatile long rightEncPos = 0;

// Constructor: Assigns pin configurations
TwoWDController::TwoWDController(int leftMotor[], int rightMotor[], int leftEnc[], int rightEnc[]) {
    memcpy(leftMotorPins, leftMotor, sizeof(leftMotorPins));
    memcpy(rightMotorPins, rightMotor, sizeof(rightMotorPins));
    memcpy(leftEncPins, leftEnc, sizeof(leftEncPins));
    memcpy(rightEncPins, rightEnc, sizeof(rightEncPins));
}

// Initialize pins and encoders
void TwoWDController::begin() {
    for (int i = 0; i < 3; i++) {
        pinMode(leftMotorPins[i], OUTPUT);
        pinMode(rightMotorPins[i], OUTPUT);
    }
    
    for (int i = 0; i < 2; i++) {
        pinMode(leftEncPins[i], INPUT_PULLUP);
        pinMode(rightEncPins[i], INPUT_PULLUP);
    }

    attachInterrupt(digitalPinToInterrupt(leftEncPins[0]), leftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(leftEncPins[1]), leftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(rightEncPins[0]), rightEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(rightEncPins[1]), rightEncoderISR, CHANGE);

    resetPID();
}

// ISR for left encoder
void IRAM_ATTR TwoWDController::leftEncoderISR() {
    static uint8_t lastState = 0;
    lastState = (lastState << 2) | ((digitalRead(leftEncPins[0]) << 1) | digitalRead(leftEncPins[1]));
    leftEncPos += ENC_STATES[lastState & 0x0F];
}

// ISR for right encoder
void IRAM_ATTR TwoWDController::rightEncoderISR() {
    static uint8_t lastState = 0;
    lastState = (lastState << 2) | ((digitalRead(rightEncPins[0]) << 1) | digitalRead(rightEncPins[1]));
    rightEncPos += ENC_STATES[lastState & 0x0F];
}

// Set motor speeds (scaled -255 to 255)
void TwoWDController::drive(double leftSpeed, double rightSpeed) {
    leftPID.TargetTicksPerFrame = leftSpeed;
    rightPID.TargetTicksPerFrame = rightSpeed;
}

// Update PID and control motors
void TwoWDController::updatePID() {
    doPID(&leftPID);
    doPID(&rightPID);
    setMotorSpeeds(leftPID.output, rightPID.output);
}

// PID Controller
void TwoWDController::doPID(PID* p) {
    long P, I, D;
    long error = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);
    p->PrevEnc = p->Encoder;
    
    P = error;  
    I = p->ITerm + error;
    D = error - p->PrevInput;

    p->output = (P * 1.2) + (I * 0.01) + (D * 0.8);
    p->ITerm = I;
    p->PrevInput = error;

    if (p->output > 255) p->output = 255;
    if (p->output < -255) p->output = -255;
}

// Set motor speeds (PWM control)
void TwoWDController::setMotorSpeeds(int leftSpeed, int rightSpeed) {
    // Left motor control
    digitalWrite(leftMotorPins[0], leftSpeed > 0);
    digitalWrite(leftMotorPins[1], leftSpeed < 0);
    analogWrite(leftMotorPins[2], abs(leftSpeed));

    // Right motor control
    digitalWrite(rightMotorPins[0], rightSpeed > 0);
    digitalWrite(rightMotorPins[1], rightSpeed < 0);
    analogWrite(rightMotorPins[2], abs(rightSpeed));
}

// Read encoder values
long TwoWDController::readEncoder(int i) {
    return (i == 0) ? leftEncPos : rightEncPos;
}

// Reset encoders
void TwoWDController::resetEncoders() {
    leftEncPos = 0;
    rightEncPos = 0;
}

// Reset PID values
void TwoWDController::resetPID() {
    leftPID = {0, 0, 0, 0, 0, 0};
    rightPID = {0, 0, 0, 0, 0, 0};
}

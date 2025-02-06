#include "PIDControl.h"

// Constructor
PIDControl::PIDControl() {
  resetPID();
}

// Reset PID parameters
void PIDControl::resetPID() {
  resetEncoders();  // Reset encoders using encoder module
  leftPID = { 0, 0, 0, 0, 0, 0 };
  rightPID = { 0, 0, 0, 0, 0, 0 };
}

// Set desired speed (in meters per second)
void PIDControl::drive(double leftSpeed, double rightSpeed) {
  double leftRevsPerSecond = leftSpeed / (WHEEL_DIAMETER * 3.14159265359);
  double rightRevsPerSecond = rightSpeed / (WHEEL_DIAMETER * 3.14159265359);

  leftPID.TargetTicksPerFrame = leftRevsPerSecond * ENCODER_RESOLUTION * PID_INTERVAL * GEAR_REDUCTION;
  rightPID.TargetTicksPerFrame = rightRevsPerSecond * ENCODER_RESOLUTION * PID_INTERVAL * GEAR_REDUCTION;
}

// Calculate speed in meters per second
double PIDControl::getSpeedMetersPerSecond(long encoderTicks) {
  double wheelCircumference = WHEEL_DIAMETER * PI;                   // Circumference in meters
  double distancePerTick = wheelCircumference / ENCODER_RESOLUTION;  // Distance per tick in meters
  double speed = (encoderTicks * distancePerTick) / PID_INTERVAL;    // Speed in meters per second
  return speed;
}

// Update PID control loop by reading encoder values
void PIDControl::updatePID() {
  leftPID.Encoder = readEncoder(0);   // Read left encoder
  rightPID.Encoder = readEncoder(1);  // Read right encoder

  computePID(leftPID);
  computePID(rightPID);
}

// Compute PID output for a given motor
void PIDControl::computePID(SetPointInfo &pid) {
  int input = pid.Encoder - pid.PrevEnc;
  long Perror = pid.TargetTicksPerFrame - input;
  long computedOutput = (Kp * Perror - Kd * (input - pid.PrevInput) + pid.ITerm) / Ko;

  pid.PrevEnc = pid.Encoder;

  // Apply constraints to output
  computedOutput += pid.output;
  if (computedOutput >= MAX_PWM)
    computedOutput = MAX_PWM;
  else if (computedOutput <= -MAX_PWM)
    computedOutput = -MAX_PWM;
  else
    pid.ITerm += Ki * Perror;

  pid.output = computedOutput;
  pid.PrevInput = input;
}

// Get computed motor outputs
int PIDControl::getLeftOutput() const {
  return leftPID.output;
}

int PIDControl::getRightOutput() const {
  return rightPID.output;
}

int PIDControl::getPrevInput(int i) const {  //0 LEFT, 1 RIGHT
  return (i > 0 ? rightPID.PrevInput : leftPID.PrevInput);
}
int PIDControl::getOutput(int i) const {
  return (i > 0 ? rightPID.output : leftPID.output);
}
double PIDControl::getTargetTicksPerFrame(int i) const {
  return (i > 0 ? rightPID.TargetTicksPerFrame : leftPID.TargetTicksPerFrame);
}
double PIDControl::getSpeedRevPerMinute(int encPrevInput){
  
}

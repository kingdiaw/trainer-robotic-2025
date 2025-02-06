#ifndef PIDCONTROL_H
#define PIDCONTROL_H

#include "encoder.h"  // Include encoder header
#include "DriverL298.h"

struct SetPointInfo {
  double TargetTicksPerFrame;  // Desired speed in ticks per frame
  long Encoder;                // Current encoder count
  long PrevEnc;                // Previous encoder count
  int PrevInput;               // Previous input (to avoid derivative kick)
  int ITerm;                   // Integral term
  long output;                 // Last motor setting
};

class PIDControl {
public:
  // Constructor
  PIDControl();

  // Reset PID parameters
  void resetPID();

  // Update PID by reading encoder values
  void updatePID();

  // Set desired speed for both motors
  void drive(double leftSpeed, double rightSpeed);

  // Get the computed motor output
  int getLeftOutput() const;
  int getRightOutput() const;
  int getPrevInput(int i) const;
  int getOutput(int i) const;
  double getTargetTicksPerFrame(int i) const;
  double getSpeedMetersPerSecond(long encoderTicks);
  double getSpeedRevPerMinute(int i);
    // Robot parameters
    static constexpr double WHEEL_DIAMETER = 0.067;
  static constexpr int ENCODER_RESOLUTION = 3950;
  static constexpr double PID_INTERVAL = 0.033;
  static constexpr int GEAR_REDUCTION = 1;

  // PID parameters
  static constexpr int Kp = 20;
  static constexpr int Kd = 12;
  static constexpr int Ki = 0;
  static constexpr int Ko = 50;
  static constexpr int MAX_PWM = 255;

  // PID structures for left and right motors
  SetPointInfo leftPID;
  SetPointInfo rightPID;

  // PID computation
  void computePID(SetPointInfo &pid);
};

#endif  // PIDCONTROL_H

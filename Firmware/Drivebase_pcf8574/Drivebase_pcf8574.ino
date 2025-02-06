#include <Arduino.h>
#include <Ticker.h>
#include "PCF8574.h"

// Encoder pins
#define LEFT_ENC_PIN_A 18  // GPIO18
#define LEFT_ENC_PIN_B 19  // GPIO19
// #define RIGHT_ENC_PIN_A 16  // GPIO16 (commented out)
// #define RIGHT_ENC_PIN_B 17  // GPIO17 (commented out)

// Motor pins for L298N
#define LEFT_MOTOR_FORWARD 5
#define LEFT_MOTOR_BACKWARD 25
#define LEFT_MOTOR_ENABLE 2
// #define RIGHT_MOTOR_FORWARD 14  // (commented out)
// #define RIGHT_MOTOR_BACKWARD 23 // (commented out)
// #define RIGHT_MOTOR_ENABLE 27   // (commented out)

// Robot parameters
const double WHEEL_DIAMETER = 0.067;  // Wheel diameter in meters (updated)
const int ENCODER_RESOLUTION = 3950;  // Encoder ticks per revolution
const double PID_INTERVAL = 0.033;    // PID control loop interval in seconds (30 Hz)
const int GEAR_REDUCTION = 1;         // Gear ratio (1 if no gearbox)

// PID parameters
const int Kp = 20;
const int Kd = 12;
const int Ki = 0;
const int Ko = 50;
const int MAX_PWM = 255;  // Maximum PWM value

// Encoder variables
volatile long left_enc_pos = 0L;
// volatile long right_enc_pos = 0L; // (commented out)

long lastMillis;
const int TIMER_INTERVAL = 33;

// Encoder lookup table
static const int8_t ENC_STATES[] = {
  0, 1, -1, 0,
  -1, 0, 0, 1,
  1, 0, 0, -1,
  0, -1, 1, 0
};

// PID structures
struct SetPointInfo {
  double TargetTicksPerFrame;  // Desired speed in ticks per frame
  long Encoder;                // Current encoder count
  long PrevEnc;                // Previous encoder count
  int PrevInput;               // Previous input (to avoid derivative kick)
  int ITerm;                   // Integral term
  long output;                 // Last motor setting
};

SetPointInfo leftPID;  // Only left motor PID
// SetPointInfo rightPID; // (commented out)

//Creating Object
Ticker repeat;
PCF8574 pcf1(0x20);

// Function prototypes
void resetPID();
void doPID(SetPointInfo* p);
void updatePID();
void setMotorSpeeds(int leftSpeed, int rightSpeed);
void drive(double leftSpeed, double rightSpeed);
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();
double getSpeedMetersPerSecond(long encoderTicks);

// ISR for LEFT encoder
void IRAM_ATTR leftEncoderISR() {
  static uint8_t enc_last = 0;
  enc_last <<= 2;                                                                  // Shift previous state two places
  enc_last |= ((digitalRead(LEFT_ENC_PIN_A) << 1) | digitalRead(LEFT_ENC_PIN_B));  // Read the current state
  left_enc_pos += ENC_STATES[(enc_last & 0x0F)];
}

// ISR for RIGHT encoder (commented out)
/*
void IRAM_ATTR rightEncoderISR() {
  static uint8_t enc_last = 0;
  enc_last <<= 2; // Shift previous state two places
  enc_last |= ((digitalRead(RIGHT_ENC_PIN_A) << 1) | digitalRead(RIGHT_ENC_PIN_B)); // Read the current state
  right_enc_pos += ENC_STATES[(enc_last & 0x0F)];
}
*/

// Wrap the encoder reading function
long readEncoder(int i) {
  if (i == 0) return left_enc_pos;  // LEFT encoder
  // else return right_enc_pos; // RIGHT encoder (commented out)
  return 0;  // Default return value
}

// Wrap the encoder reset function
void resetEncoder(int i) {
  if (i == 0) {
    left_enc_pos = 0L;
  }
  // else {
  //   right_enc_pos = 0L; // (commented out)
  // }
}

// Reset both encoders
void resetEncoders() {
  resetEncoder(0);
  // resetEncoder(1); // (commented out)
}

// Calculate speed in meters per second
double getSpeedMetersPerSecond(long encoderTicks) {
  double wheelCircumference = WHEEL_DIAMETER * PI;                   // Circumference in meters
  double distancePerTick = wheelCircumference / ENCODER_RESOLUTION;  // Distance per tick in meters
  double speed = (encoderTicks * distancePerTick) / PID_INTERVAL;    // Speed in meters per second
  return speed;
}

// Reset PID variables
void resetPID() {
  leftPID.TargetTicksPerFrame = 0.0;
  leftPID.Encoder = readEncoder(0);
  leftPID.PrevEnc = leftPID.Encoder;
  leftPID.output = 0;
  leftPID.PrevInput = 0;
  leftPID.ITerm = 0;

  // rightPID.TargetTicksPerFrame = 0.0; // (commented out)
  // rightPID.Encoder = readEncoder(1); // (commented out)
  // rightPID.PrevEnc = rightPID.Encoder; // (commented out)
  // rightPID.output = 0; // (commented out)
  // rightPID.PrevInput = 0; // (commented out)
  // rightPID.ITerm = 0; // (commented out)
}

// PID control logic
void doPID(SetPointInfo* p) {
  long Perror;
  long output;
  int input;

  // Calculate error
  input = p->Encoder - p->PrevEnc;
  Perror = p->TargetTicksPerFrame - input;

  // Compute PID output
  output = (Kp * Perror - Kd * (input - p->PrevInput) + p->ITerm) / Ko;
  p->PrevEnc = p->Encoder;

  // Accumulate integral term or clamp output
  output += p->output;
  if (output >= MAX_PWM)
    output = MAX_PWM;
  else if (output <= -MAX_PWM)
    output = -MAX_PWM;
  else
    p->ITerm += Ki * Perror;

  p->output = output;
  p->PrevInput = input;
}

// Update PID control for both motors
void updatePID() {
  // Read encoders
  leftPID.Encoder = readEncoder(0);
  // rightPID.Encoder = readEncoder(1); // (commented out)

  // Compute PID for both motors
  doPID(&leftPID);
  // doPID(&rightPID); // (commented out)

  // Set motor speeds
  setMotorSpeeds(leftPID.output, 0);  // Right motor speed set to 0
}

// Set motor speeds using L298N motor driver
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  // Left motor
  if (leftSpeed >= 0) {
    //digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
    pcf1.digitalWrite(P0, HIGH);
    pcf1.digitalWrite(P1, LOW);
  } else {
    pcf1.digitalWrite(P0, LOW);
    pcf1.digitalWrite(P1, HIGH);
    leftSpeed = -leftSpeed;
  }
  analogWrite(LEFT_MOTOR_ENABLE, constrain(leftSpeed, 0, 255));

  // Right motor (commented out)
  /*
  if (rightSpeed >= 0) {
    pcf1.digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
    pcf1.digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
  } else {
    pcf1.digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
    pcf1.digitalWrite(RIGHT_MOTOR_BACKWARD, HIGH);
    rightSpeed = -rightSpeed;
  }
  analogWrite(RIGHT_MOTOR_ENABLE, constrain(rightSpeed, 0, 255));
  */
}

// Drive the robot at a given speed in meters per second
//Spec Motor JGB37-520:
//Max RPM=92 (at 12V)
//Encoder Max TickPerFrame (30Hz) = 200
//Encoder Resolution = 3950
void drive(double leftSpeed, double rightSpeed) {
  // Convert speed from m/s to encoder ticks per PID interval
  double leftRevsPerSecond = leftSpeed / (WHEEL_DIAMETER * PI);
  // double rightRevsPerSecond = rightSpeed / (WHEEL_DIAMETER * PI); // (commented out)

  leftPID.TargetTicksPerFrame = leftRevsPerSecond * ENCODER_RESOLUTION * PID_INTERVAL * GEAR_REDUCTION;
  // rightPID.TargetTicksPerFrame = rightRevsPerSecond * ENCODER_RESOLUTION * PID_INTERVAL * GEAR_REDUCTION; // (commented out)
}

// Setup function
void setup() {
  // Configure encoder pins
  pinMode(LEFT_ENC_PIN_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_PIN_B, INPUT_PULLUP);
  // pinMode(RIGHT_ENC_PIN_A, INPUT_PULLUP); // (commented out)
  // pinMode(RIGHT_ENC_PIN_B, INPUT_PULLUP); // (commented out)

  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_B), leftEncoderISR, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN_A), rightEncoderISR, CHANGE); // (commented out)
  // attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN_B), rightEncoderISR, CHANGE); // (commented out)

  // Initialize motor pins
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(LEFT_MOTOR_ENABLE, OUTPUT);
  // pinMode(RIGHT_MOTOR_FORWARD, OUTPUT); // (commented out)
  // pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT); // (commented out)
  // pinMode(RIGHT_MOTOR_ENABLE, OUTPUT); // (commented out)

  // Initialize PID
  resetPID();

  // Start serial communication for debugging
  Serial.begin(115200);

  // updatePID every 33ms
  repeat.attach(0.033, updatePID);
  Serial.println(pcf1.begin() > 0 ? "OK" : "FAIL");
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pcf1.pinMode(P0, OUTPUT);
  pcf1.pinMode(P1, OUTPUT);
  pcf1.pinMode(P2, OUTPUT);
  pcf1.pinMode(P3, OUTPUT);


  // Example: Drive the left motor forward at 0.33 m/s
  drive(0.12, 0);  // Right motor speed set to 0
  Serial.println("Prog Start!");
}

long debugTick;


// Main loop
void loop() {

  if (millis() > debugTick) {
    debugTick = millis() + 500;
    // Debugging: Print encoder values and speeds
    long leftTicks = readEncoder(0);
    // long rightTicks = readEncoder(1); // (commented out)

    double leftSpeed = getSpeedMetersPerSecond(leftPID.PrevInput);

    Serial.print("Left Speed (m/s): ");
    Serial.print(leftSpeed);
    Serial.print(" | PWM: ");
    Serial.print(leftPID.output);
    Serial.print(" | TargetTPF: ");
    Serial.print(leftPID.TargetTicksPerFrame);
    Serial.print(" | ActualTPF: ");
    Serial.print(leftPID.PrevInput);
    Serial.print(" | Revs/min: ");
    Serial.print((leftPID.PrevInput) / (ENCODER_RESOLUTION * PID_INTERVAL) * 60);

    // Serial.print(" | Right Speed (m/s): "); // (commented out)
    // Serial.println(rightSpeed); // (commented out)
    Serial.println();
  }
}
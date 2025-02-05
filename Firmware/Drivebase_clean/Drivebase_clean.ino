#include <Arduino.h>

// Encoder pins
#define LEFT_ENC_PIN_A 18  // GPIO18
#define LEFT_ENC_PIN_B 19  // GPIO19
#define RIGHT_ENC_PIN_A 16  // GPIO16
#define RIGHT_ENC_PIN_B 17  // GPIO17

// Motor pins for L298N
#define LEFT_MOTOR_FORWARD 5
#define LEFT_MOTOR_BACKWARD 25
#define LEFT_MOTOR_ENABLE 2
#define RIGHT_MOTOR_FORWARD 14
#define RIGHT_MOTOR_BACKWARD 23
#define RIGHT_MOTOR_ENABLE 27

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
volatile long right_enc_pos = 0L;

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
  double TargetTicksPerFrame;
  long Encoder;
  long PrevEnc;
  int PrevInput;
  int ITerm;
  long output;
};

SetPointInfo leftPID;
SetPointInfo rightPID;

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
  enc_last <<= 2;
  enc_last |= ((digitalRead(LEFT_ENC_PIN_A) << 1) | digitalRead(LEFT_ENC_PIN_B));
  left_enc_pos += ENC_STATES[(enc_last & 0x0F)];
}

// ISR for RIGHT encoder
void IRAM_ATTR rightEncoderISR() {
  static uint8_t enc_last = 0;
  enc_last <<= 2;
  enc_last |= ((digitalRead(RIGHT_ENC_PIN_A) << 1) | digitalRead(RIGHT_ENC_PIN_B));
  right_enc_pos += ENC_STATES[(enc_last & 0x0F)];
}

long readEncoder(int i) {
  if (i == 0) return left_enc_pos;
  else return right_enc_pos;
}

void resetEncoder(int i) {
  if (i == 0) {
    left_enc_pos = 0L;
  } else {
    right_enc_pos = 0L;
  }
}

void resetEncoders() {
  resetEncoder(0);
  resetEncoder(1);
}

void resetPID() {
  leftPID.TargetTicksPerFrame = 0.0;
  leftPID.Encoder = readEncoder(0);
  leftPID.PrevEnc = leftPID.Encoder;
  leftPID.output = 0;
  leftPID.PrevInput = 0;
  leftPID.ITerm = 0;

  rightPID.TargetTicksPerFrame = 0.0;
  rightPID.Encoder = readEncoder(1);
  rightPID.PrevEnc = rightPID.Encoder;
  rightPID.output = 0;
  rightPID.PrevInput = 0;
  rightPID.ITerm = 0;
}

void updatePID() {
  leftPID.Encoder = readEncoder(0);
  rightPID.Encoder = readEncoder(1);

  doPID(&leftPID);
  doPID(&rightPID);

  setMotorSpeeds(leftPID.output, rightPID.output);
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  if (leftSpeed >= 0) {
    digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
    digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  } else {
    digitalWrite(LEFT_MOTOR_FORWARD, LOW);
    digitalWrite(LEFT_MOTOR_BACKWARD, HIGH);
    leftSpeed = -leftSpeed;
  }
  analogWrite(LEFT_MOTOR_ENABLE, constrain(leftSpeed, 0, 255));

  if (rightSpeed >= 0) {
    digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
    digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
  } else {
    digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
    digitalWrite(RIGHT_MOTOR_BACKWARD, HIGH);
    rightSpeed = -rightSpeed;
  }
  analogWrite(RIGHT_MOTOR_ENABLE, constrain(rightSpeed, 0, 255));
}

void drive(double leftSpeed, double rightSpeed) {
  double leftRevsPerSecond = leftSpeed / (WHEEL_DIAMETER * PI);
  double rightRevsPerSecond = rightSpeed / (WHEEL_DIAMETER * PI);

  leftPID.TargetTicksPerFrame = leftRevsPerSecond * ENCODER_RESOLUTION * PID_INTERVAL * GEAR_REDUCTION;
  rightPID.TargetTicksPerFrame = rightRevsPerSecond * ENCODER_RESOLUTION * PID_INTERVAL * GEAR_REDUCTION;
}

void setup() {
  pinMode(LEFT_ENC_PIN_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_PIN_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_PIN_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_PIN_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_B), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN_A), rightEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN_B), rightEncoderISR, CHANGE);

  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(LEFT_MOTOR_ENABLE, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_ENABLE, OUTPUT);

  resetPID();
  Serial.begin(115200);
  drive(0.3, 0.3);
}

void loop() {
  if (millis() - lastMillis >= TIMER_INTERVAL) {
    lastMillis = millis();
    updatePID();
  }
}

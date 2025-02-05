#include <Arduino.h>

#define LEFT_ENC_PIN_A 18   // GPIO16
#define LEFT_ENC_PIN_B 19   // GPIO17
#define LEFT_MOTOR_FORWARD 5
#define LEFT_MOTOR_BACKWARD 25
#define LEFT_MOTOR_ENABLE 2
#define LEFT  0
#define RIGHT 1
#define PPR 3950  // Pulses Per Revolution
#define TIMER_INTERVAL 33  // Time interval in milliseconds (50ms)

/* PID setpoint info For a Motor */
typedef struct {
  double TargetTicksPerFrame;    // target speed in ticks per frame
  long Encoder;                  // encoder count
  long PrevEnc;                  // last encoder count

  /*
  * Using previous input (PrevInput) instead of PrevError to avoid derivative kick,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  */
  int PrevInput;                // last input
  //int PrevErr;                   // last error

  /*
  * Using integrated term (ITerm) instead of integrated error (Ierror),
  * to allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  //int Ierror;
  int ITerm;                    //integrated term

  long output;                    // last motor setting
}
SetPointInfo;

SetPointInfo leftPID, rightPID;

/* PID Parameters */
int Kp = 20;
int Kd = 12;
int Ki = 0;
int Ko = 50;

unsigned char moving = 0; // is the base in motion?

volatile long left_enc_pos = 0L;
volatile long last_left_enc_pos = 0L;
float target_rpm = 25.0; // Desired speed in RPM
float kp = 2.0; // Proportional gain
float ki = 0.1; // Integral gain
float integral_error = 0.0; // Cumulative error

unsigned long lastMillis = 0;

void IRAM_ATTR leftEncoderISR() {
  static uint8_t enc_last = 0;
  enc_last <<= 2;
  enc_last |= ((digitalRead(LEFT_ENC_PIN_A) << 1) | digitalRead(LEFT_ENC_PIN_B));
  left_enc_pos += (enc_last & 0x03) - 1;
}

long readEncoder() {
  return left_enc_pos;
}

void resetEncoder() {
  left_enc_pos = 0L;
}

void setMotorSpeed(int speed) {
  if (speed < 0) {
    speed = -speed;
    digitalWrite(LEFT_MOTOR_FORWARD, LOW);
    digitalWrite(LEFT_MOTOR_BACKWARD, HIGH);
  } else {
    digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
    digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  }
  analogWrite(LEFT_MOTOR_ENABLE, constrain(speed, 0, 255));
}

void calculateSpeed() {
  long current_pos = readEncoder();
  long delta_pos = current_pos - last_left_enc_pos;
  last_left_enc_pos = current_pos;
  float rpm = (delta_pos / (float)PPR) * (60000.0 / TIMER_INTERVAL);

  float error = target_rpm - rpm;
  integral_error += error; // Accumulate the error over time

  // Calculate the PWM output using PI control
  int pwm_output = constrain((int)(kp * error + ki * integral_error), 0, 255);
  setMotorSpeed(pwm_output);

  Serial.print("RPM: "); Serial.print(rpm);
  Serial.print(" | Error: "); Serial.print(error);
  Serial.print(" | Integral Error: "); Serial.print(integral_error);
  Serial.print(" | PWM: "); Serial.println(pwm_output);
}

void resetPID(){
   leftPID.TargetTicksPerFrame = 0.0;
   leftPID.Encoder = readEncoder();
   leftPID.PrevEnc = leftPID.Encoder;
   leftPID.output = 0;
   leftPID.PrevInput = 0;
   leftPID.ITerm = 0;
}

/* PID routine to compute the next motor commands */
void doPID(SetPointInfo * p) {
  long Perror;
  long output;
  int input;

  //Perror = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);
  input = p->Encoder - p->PrevEnc;
  Perror = p->TargetTicksPerFrame - input;


  /*
  * Avoid derivative kick and allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  //output = (Kp * Perror + Kd * (Perror - p->PrevErr) + Ki * p->Ierror) / Ko;
  // p->PrevErr = Perror;
  output = (Kp * Perror - Kd * (input - p->PrevInput) + p->ITerm) / Ko;
  p->PrevEnc = p->Encoder;

  output += p->output;
  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates
  if (output >= 255)
    output = 255;
  else if (output <= -255)
    output = -255;
  else
  /*
  * allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
    p->ITerm += Ki * Perror;

  p->output = output;
  p->PrevInput = input;
}

/* Read the encoder values and call the PID routine */
void updatePID() {
  /* Read the encoders */
  leftPID.Encoder = readEncoder();
  

  /* Compute PID update for each motor */
  doPID(&leftPID);

  /* Set the motor speeds accordingly */
  setMotorSpeed(leftPID.output);

  //debug message
  long current_pos = leftPID.Encoder;
  long delta_pos = current_pos - last_left_enc_pos;
  last_left_enc_pos = current_pos;
  float rpm = (delta_pos / (float)PPR) * (60000.0 / TIMER_INTERVAL);

  Serial.print("RPM: "); Serial.print(rpm);
  Serial.print(" | PWM: "); Serial.println(leftPID.output);
}

void setup() {
  Serial.begin(115200);
  pinMode(LEFT_ENC_PIN_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_PIN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_B), leftEncoderISR, CHANGE);
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(LEFT_MOTOR_ENABLE, OUTPUT);
  
  leftPID.TargetTicksPerFrame = 65;
  Serial.print("Target:"); Serial.print(leftPID.TargetTicksPerFrame);Serial.print(" | PWM: "); Serial.println(leftPID.output);

}

void loop() {
  if (millis() - lastMillis >= TIMER_INTERVAL) {
    lastMillis = millis();
    updatePID();
    //calculateSpeed();
  }
}
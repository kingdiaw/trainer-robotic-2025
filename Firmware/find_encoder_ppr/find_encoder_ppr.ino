//reference:https://chatgpt.com/share/678a15b8-0388-8008-8f2e-60875a33929b
#include <Arduino.h>

#define LEFT_ENC_PIN_A 18   // GPIO16
#define LEFT_ENC_PIN_B 19   // GPIO17
#define RIGHT_ENC_PIN_A 16  // GPIO18
#define RIGHT_ENC_PIN_B 17  // GPIO19
#define LEFT_MOTOR_FORWARD 5
#define LEFT_MOTOR_BACKWARD 25
#define LEFT_MOTOR_ENABLE 2

#define PPR   3950

#define LEFT 1
#define RIGHT 2
// Prototype Functions
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);

volatile long left_enc_pos = 0L;
volatile long right_enc_pos = 0L;

// Encoder lookup table
static const int8_t ENC_STATES[] = {
  0, 1, -1, 0,
  -1, 0, 0, 1,
  1, 0, 0, -1,
  0, -1, 1, 0
};

// ISR for LEFT encoder
void IRAM_ATTR leftEncoderISR() {
  static uint8_t enc_last = 0;
  enc_last <<= 2;                                                                  // Shift previous state two places
  enc_last |= ((digitalRead(LEFT_ENC_PIN_A) << 1) | digitalRead(LEFT_ENC_PIN_B));  // Read the current state
  left_enc_pos += ENC_STATES[(enc_last & 0x0F)];
}

// ISR for RIGHT encoder
void IRAM_ATTR rightEncoderISR() {
  static uint8_t enc_last = 0;
  enc_last <<= 2;                                                                    // Shift previous state two places
  enc_last |= ((digitalRead(RIGHT_ENC_PIN_A) << 1) | digitalRead(RIGHT_ENC_PIN_B));  // Read the current state
  right_enc_pos += ENC_STATES[(enc_last & 0x0F)];
}

// Wrap the encoder reading function
long readEncoder(int i) {
  if (i == 0) return left_enc_pos;  // LEFT encoder
  else return right_enc_pos;        // RIGHT encoder
}

// Wrap the encoder reset function
void resetEncoder(int i) {
  if (i == 0) {
    left_enc_pos = 0L;
  } else {
    right_enc_pos = 0L;
  }
}

// Reset both encoders
void resetEncoders() {
  resetEncoder(0);
  resetEncoder(1);
}

void setMotorSpeed(int i, int spd) {
  unsigned char reverse = 0;

  if (spd < 0) {
    spd = -spd;
    reverse = 1;
  }
  if (spd > 255)
    spd = 255;

  if (i == LEFT) {
    if (reverse == 0) {
      digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
      digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
    } else if (reverse == 1) {
      digitalWrite(LEFT_MOTOR_FORWARD, LOW);
      digitalWrite(LEFT_MOTOR_BACKWARD, HIGH);
    }
    analogWrite(LEFT_MOTOR_ENABLE, spd);
  } else /*if (i == RIGHT) //no need for condition*/ {
    // if      (reverse == 0) { digatalWrite(RIGHT_MOTOR_FORWARD, HIGH); digitalWrite(RIGHT_MOTOR_BACKWARD, LOW); }
    //else if (reverse == 1) { digitalWrite(RIGHT_MOTOR_BACKWARD, LOW); digitalWrite(RIGHT_MOTOR_FORWARD, HIGH); }
  }
}

//Global Variable
int t;
String input = "";  // Variable to store incoming data

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

  pinMode(5, OUTPUT);
  pinMode(2, OUTPUT);

  digitalWrite(5, LOW);

  while (true) {
    input = Serial.readStringUntil('\n');
    input.trim();       // Remove leading/trailing whitespace or newline
    t = input.toInt();  // Convert the string to an integer
    if (t > 10) {
      Serial.print(t);
      Serial.println("ms");
      break;
    }
  }
  Serial.println("Loop start!");
  digitalWrite(5, HIGH);
  analogWrite(2,85);
  delay(t);
  analogWrite(2,50);
}

void loop() {
  // Example usage
  do{}while(readEncoder(0)<3950*2);
  digitalWrite(5, LOW);
  analogWrite(2,255);
  for(byte i=0;i<4;i++){
  Serial.println("MOTOR STOP!");
  Serial.print("Left Encoder: ");
  Serial.println(readEncoder(0));
  Serial.print("Right Encoder: ");
  Serial.println(readEncoder(1));
  delay(500);
  }

  while (true)
    ;
}

//reference:https://chatgpt.com/share/678a15b8-0388-8008-8f2e-60875a33929b
#include <Arduino.h>

#define LEFT_ENC_PIN_A 16   // GPIO16
#define LEFT_ENC_PIN_B 17   // GPIO17
#define RIGHT_ENC_PIN_A 18  // GPIO18
#define RIGHT_ENC_PIN_B 19  // GPIO19

// Prototype Functions
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();

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

void setup() {
  Serial.begin(115200);
  delay(1000);
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
}

void loop() {
  // Example usage


  Serial.print("Left Encoder: ");
  Serial.println(readEncoder(0));
  Serial.print("Right Encoder: ");
  Serial.println(readEncoder(1));

  delay(500);  // Simulate main loop delay
}

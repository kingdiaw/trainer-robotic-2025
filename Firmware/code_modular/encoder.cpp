#include "encoder.h"

volatile long left_enc_pos = 0L;
volatile long right_enc_pos = 0L;

volatile static uint8_t left_enc_last = 0;
volatile static uint8_t right_enc_last = 0;

// ISR for LEFT encoder
void IRAM_ATTR leftEncoderISR() {
  left_enc_last <<= 2;
  left_enc_last |= ((digitalRead(LEFT_ENC_PIN_A) << 1) | digitalRead(LEFT_ENC_PIN_B));
  left_enc_pos += ENC_STATES[(left_enc_last & 0x0F)];
}

// ISR for RIGHT encoder
void IRAM_ATTR rightEncoderISR() {
  right_enc_last <<= 2;
  right_enc_last |= ((digitalRead(RIGHT_ENC_PIN_A) << 1) | digitalRead(RIGHT_ENC_PIN_B));
  right_enc_pos += ENC_STATES[(right_enc_last & 0x0F)];
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
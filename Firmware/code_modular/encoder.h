#ifndef ENCODER_h
#define ENCODER_h

#include <Arduino.h>

#define LEFT_ENC_PIN_A 18   // GPIO18
#define LEFT_ENC_PIN_B 19   // GPIO19
#define RIGHT_ENC_PIN_A 16  // GPIO16
#define RIGHT_ENC_PIN_B 17  // GPIO17

// Declare the encoder positions as extern
extern volatile long left_enc_pos;
extern volatile long right_enc_pos;

// Function prototypes
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();

// Declare ISRs
void IRAM_ATTR leftEncoderISR();
void IRAM_ATTR rightEncoderISR();

// Encoder lookup table
static const int8_t ENC_STATES[] = {
  0, 1, -1, 0,
  -1, 0, 0, 1,
  1, 0, 0, -1,
  0, -1, 1, 0
};

#endif
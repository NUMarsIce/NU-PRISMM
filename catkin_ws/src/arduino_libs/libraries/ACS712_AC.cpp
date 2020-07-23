/*
 * ACS712 Class implementation
 */

#include "headers/ACS712_AC.h"

ACS712_AC::ACS712_AC(int pin, int mVperAmp) {
  buffer_index = 0;
  analog_pin_ = pin;
  mVperAmp_ = mVperAmp;
  pinMode(pin, INPUT);
}

ACS712_AC::~ACS712_AC(){}

float ACS712_AC::read() {
  float RawValue = analogRead(analog_pin_);
  float Voltage = (RawValue / 1024.0) * 5000;  // Gets you mV
  float sample_current = ((Voltage - ACS_OFFSET_) / mVperAmp_);

  // Update the buffer
  rolling_buf[buffer_index] = sample_current;

  // Find the mean of squares
  float sqsum = 0;
  for (int i = 0; i < 30; i++) {
    sqsum = sqsum + (pow(rolling_buf[i], 2));
  }

  // find the square root of that mean
  float ret = sqrt((sqsum / 30));

  // update the roll index
  if (buffer_index >= (29)) {
    buffer_index = 0;
  } else {
    buffer_index++;
  }

  // Return the result
  return ret;
}

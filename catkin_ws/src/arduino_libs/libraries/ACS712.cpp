/*
 * ACS712 Class implementation
 */

#include "headers/ACS712.h"
ACS712::ACS712(int pin) { analog_pin_ = pin; }

ACS712::~ACS712(){}

float ACS712::read() {
  float RawValue = analogRead(analog_pin_);
  float Voltage = (RawValue / 1024.0) * 5000;  // Gets you mV
  return ((Voltage - ACS_OFFSET_) / mVperAmp_);
}

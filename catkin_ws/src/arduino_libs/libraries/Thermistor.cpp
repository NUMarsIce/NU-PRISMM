/*
 * Thermistor.cpp
 * Source code for simple thermistor
 *
 */

#include "headers/Thermistor.h"

Thermistor::Thermistor(uint8_t pin, uint16_t b_coef, uint16_t therm_res, uint16_t series_res) {
  pin_ = pin;
  b_coef_ = b_coef;
  therm_res_ = therm_res;
  series_res_ = series_res;

  // Setup Pin modes
  pinMode(pin_, INPUT);
}

float Thermistor::read() {
  float raw_reading = analogRead(pin_);
  raw_reading = 1023 / raw_reading - 1;
  raw_reading = series_res_ / raw_reading;
  float steinhart;
  steinhart = raw_reading / therm_res_;
  steinhart = log(steinhart);
  steinhart /= b_coef_;
  steinhart += 1.0 / (25 + 273.15);
  steinhart = 1.0 / steinhart;
  steinhart -= 273.15;

  return steinhart;
}

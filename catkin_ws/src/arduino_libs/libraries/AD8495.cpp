/*
 *  AD8495 - Thermo couple breakout board
 *  Equations from : https://learn.adafruit.com/ad8495-thermocouple-amplifier/arduino
 *
 */
#include "headers/AD8495.h"

AD8495::AD8495(int pin) {
  pin_ = pin;
  pinMode(pin_, INPUT);
}

float AD8495::read() {
  float raw_adc = analogRead(pin_);
  float voltage = raw_adc * (AREF / (pow(2, ADC_RESOLUTION) - 1));
  float temp = (voltage - 1.25) / 0.005;
  return temp;
}

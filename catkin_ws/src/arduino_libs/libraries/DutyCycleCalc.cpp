#include "headers/DutyCycleCalc.h"

DutyCycleCalc::DutyCycleCalc(long period) { period_ = period; }

void DutyCycleCalc::update(bool state) {
  // If we have completed a period
  if (on_time_ + off_time_ > period_) {
    last_period_duty_cycle_ = (float)on_time_ / (on_time_ + off_time_);
    on_time_ = 0;
    off_time_ = 0;
  }

  // Update ontime and off time
  long now = millis();
  if (state) {
    on_time_ += now - last_update_millis_;
  } else {
    off_time_ += now - last_update_millis_;
  }

  // Update the last millis
  last_update_millis_ = now;
}

float DutyCycleCalc::read() { return last_period_duty_cycle_; }

#ifndef _DUTY_CYCLE_CALC_H_
#define _DUTY_CYCLE_CALC_H_

#include <Arduino.h>

class DutyCycleCalc {
 public:
  DutyCycleCalc(long period);

  // Updates the duty cycle infomation this should be called each clocl
  // Also call before the state can change in the loop
  void update(bool state);

  // Returns the duty cycle info
  float read();

 private:
  long last_update_millis_;
  long period_;
  long on_time_;
  long off_time_;
  float last_period_duty_cycle_;
};

#endif /* _DUTY_CYCLE_CALC_H_ */

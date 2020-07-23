#ifndef _ACS712_AC_H_
#define _ACS712_AC_H_

#include <Arduino.h>

class ACS712_AC {
 public:
  ACS712_AC(int pin, int mVpoerAmp = 66);
  ~ACS712_AC();
  float read();

 private:
  float rolling_buf[30];
  int buffer_index;

  int analog_pin_;
  int ACS_OFFSET_ = 2500;
  int mVperAmp_;  // use 100 for 20A Module and 66 for 30A Module
};

#endif /* _ACS_712_AC_H_ */

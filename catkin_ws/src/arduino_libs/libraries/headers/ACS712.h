#ifndef _ACS712_H_
#define _ACS712_H_

#include <Arduino.h>

class ACS712 {
 public:
  ACS712(int pin, int mVperAmp = 66);
  ~ACS712();
  float read();

 private:
  int analog_pin_;
  int ACS_OFFSET_ = 2500;
  int mVperAmp_;  // use 100 for 20A Module and 66 for 30A Module
};

#endif /* _ACS_712_H_ */

#ifndef _AD8495_H_
#define _AD8495_H_

#include <Arduino.h>

class AD8495 {
 public:
  /// Constructs a AD8495
  AD8495(int pin);

  /// Reads temperature in degrees C
  float read();

 private:
  int pin_;
  float AREF{5.0};
  int ADC_RESOLUTION{10};
};

#endif /* _AD8495_H_*/

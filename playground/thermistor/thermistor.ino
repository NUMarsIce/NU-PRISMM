#include "Thermistor.h"

Thermistor* therm;

void setup(){
  Serial.begin(115200);
  therm = new Thermistor(0, 3950, 10000, 10000);
}


void loop(){
  Serial.println(therm->read());
  delay(1000);
}

#include "AD8495.h"

AD8495 heater(A0);
AD8495 heater2(A1);

void setup(){
  Serial1.begin(9600);
  
}

void loop(){
  Serial.println(max(heater.read(), heater2.read()));
  delay(100);
}
#include <Stepper.h>
#include <MovingAverageFilter.h>


Stepper myStepper(800, 8, 9);
MovingAverageFilter stepAvg(100);
MovingAverageFilter acAvg(100);

void setup() {
  Serial.begin(115200);
  myStepper.setSpeed(1000);
}

void loop() {
  myStepper.step(8);
  
  Serial.print(stepAvg.process(-10*(((analogRead(A1) / 1024.0) * 5000 - 2500) / 100))); //Stepper DC
  Serial.print(" ");
  Serial.println(acAvg.process(10*abs((((analogRead(A0) / 1024.0) * 5000 - 2500) / 100)))); //AC
}

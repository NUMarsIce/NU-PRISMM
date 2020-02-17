#include "AccelStepper.h"

// Define some steppers and the pins the will use
AccelStepper stepper1(AccelStepper::FULL2WIRE, 22, 23);
AccelStepper stepper2(AccelStepper::FULL2WIRE, 32, 33);

void setup()
{  
    stepper1.setMaxSpeed(800.0);
    stepper1.setAcceleration(750.0);
    stepper1.moveTo(400);
    
    stepper2.setMaxSpeed(200.0);
    stepper2.setAcceleration(100.0);
    stepper2.moveTo(200);
    
}

void loop()
{
    // Change direction at the limits
    if (stepper1.distanceToGo() == 0)
        stepper1.moveTo(-stepper1.currentPosition());
    if (stepper2.distanceToGo() == 0)
        stepper2.moveTo(-stepper2.currentPosition());
    stepper1.run();
    stepper2.run();
}

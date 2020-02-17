//ROS includes
#include <ros.h>
#include <std_msgs/UInt16.h>

//Arduino includes
#include <AccelStepper.h>
#include <Arduino.h>

#define STEPS_PER_REV 800;


ros::NodeHandle  nh;

//*** Stepper position subscriber ***//
AccelStepper stepper1(AccelStepper::FULL2WIRE, 22, 23);
void stepper_cb(const std_msgs::UInt16& cmd_msg){
    stepper1.moveTo(cmd_msg.data);
}
ros::Subscriber<std_msgs::UInt16> stepper_sub("stepper_position", stepper_cb);

//*** Setup ***//
void setup() {
  // speed max is 400, but lower torque
  stepper1.setMaxSpeed(1000.0);
  stepper1.setAcceleration(750.0);
  stepper1.moveTo(400);

  nh.initNode();
  nh.subscribe(stepper_sub);
}

//*** Loop ***//
void loop(){
  nh.spinOnce();
  delay(1);
}
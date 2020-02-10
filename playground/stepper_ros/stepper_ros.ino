#include <Stepper.h>
#include <ros.h>
#include <std_msgs/UInt16.h>

const int stepsPerRevolution = 800;  // change this to fit the number of steps per revolution

ros::NodeHandle  nh;
Stepper myStepper(stepsPerRevolution, 8, 9);

void stepper_cb(const std_msgs::UInt16& cmd_msg){
    myStepper.step(cmd_msg.data);
}
ros::Subscriber<std_msgs::UInt16> sub("stepper_step", stepper_cb);

void setup() {
  // set the speed at 60 rpm:
  myStepper.setSpeed(60);

  nh.initNode();
  nh.subscribe(sub);

}

void loop(){
  nh.spinOnce();
  delay(1);
}

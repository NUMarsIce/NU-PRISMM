//ROS includes
#include <ros.h>

//Arduino includes
#include <Arduino.h>

ros::NodeHandle  nh;


//*** Setup ***//
void setup() {

  nh.initNode();
}

//*** Loop ***//
void loop(){
  nh.spinOnce();
  delay(1);
}
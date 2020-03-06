//includes
#include <AccelStepper.h>
#include <Arduino.h>
#include <Dam.h>
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <prismm_msgs/dam_data.h>

#define SEND_RATE 200

ros::NodeHandle nh;
Dam dam(nh);
char buffer[60];

//******************  Subscribers  *********************//

//*** x target position subscriber ***//
void gotoX_cb(const std_msgs::UInt16& cmd_msg){
    if(!dam.gotoX(cmd_msg.data)){
      sprintf(buffer, "Cannot set X axis target to %d", cmd_msg.data);
      nh.logwarn(buffer);
    }
}
ros::Subscriber<std_msgs::UInt16> gotoX_sub("x_axis_target", gotoX_cb);

//*** drill subscriber ***//
void drill_cb(const std_msgs::Bool& drill_msg){
    if(drill_msg.data)
      if(!dam.startDrilling())
        nh.logwarn("Cannot start drilling");
    else
      if(!dam.stopDrilling())
        nh.logwarn("Cannot stop drilling");
}
ros::Subscriber<std_msgs::Bool> drill_sub("drill", drill_cb);

//*** x axis homing subscriber ***//
void homeX_cb(const std_msgs::Empty& home_msg){
  if(!dam.homeX())
    nh.logwarn("Couldnt home X axis");
}
ros::Subscriber<std_msgs::Empty> homeX_sub("home_x_axis", homeX_cb);

//*** drill homing subscriber ***//
void homeDrill_cb(const std_msgs::Empty& home_msg){
  if(!dam.homeDrill())
    nh.logwarn("Couldnt home Drill Y axis");
}
ros::Subscriber<std_msgs::Empty> homeDrill_sub("home_drill", homeDrill_cb);

//*** x target position subscriber ***//
void gotoDrill_cb(const std_msgs::UInt16& cmd_msg){
    if(!dam.gotoDrill(cmd_msg.data)){
      sprintf(buffer, "Cannot set Drill Y axis target to %d", cmd_msg.data);
      nh.logwarn(buffer);
    }
}
ros::Subscriber<std_msgs::UInt16> gotoDrill_sub("drill_y_axis_target", gotoDrill_cb);

//*** E-Stop subscriber ***//
void eStop_cb(const std_msgs::Empty& e_stop_msg){
  if(!dam.eStop())
    nh.logwarn("Couldnt E-Stop");
}
ros::Subscriber<std_msgs::Empty> eStop_sub("e_stop", eStop_cb);

//*** Resume subscriber ***//
void resume_cb(const std_msgs::Empty& e_stop_msg){
  if(!dam.resume())
    nh.logwarn("Couldnt Resume");
}
ros::Subscriber<std_msgs::Empty> resume_sub("resume", resume_cb);

//*** Reset subscriber ***//
void reset_cb(const std_msgs::Empty& e_stop_msg){
  if(!dam.reset())
    nh.logwarn("Couldnt Reset");
}
ros::Subscriber<std_msgs::Empty> reset_sub("reset", reset_cb);

//******************  Publishers  *********************//
prismm_msgs::dam_data dam_data_msg;
ros::Publisher dam_data_pub("dam_data", &dam_data_msg);

//*** Variables ***//
long last_send = millis();

//*** Setup ***//
void setup() {
  nh.initNode();
  delay(200);

  nh.subscribe(gotoX_sub);
  nh.subscribe(drill_sub);
  nh.subscribe(homeX_sub);
  nh.subscribe(homeDrill_sub);
  nh.subscribe(gotoDrill_sub);
  nh.subscribe(eStop_sub);
  nh.subscribe(resume_sub);
  nh.subscribe(reset_sub);

  nh.advertise(dam_data_pub);
}


//*** Loop ***//
void loop(){
  nh.spinOnce();
  dam.update();

  //publish dam data
  if(millis()-last_send > SEND_RATE){
    dam_data_msg = dam.getData();
    dam_data_pub.publish(&dam_data_msg);
    last_send = millis();
  }
}
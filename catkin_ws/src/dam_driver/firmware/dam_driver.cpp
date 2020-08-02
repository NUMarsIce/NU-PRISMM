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

//*** bowl subscriber ***//
void bowl_cb(const std_msgs::Float32& bowl_msg){
    if(bowl_msg.data > 0)
      if(!dam.startBowl(bowl_msg.data))
        nh.logwarn("Cannot start bowl");
    else
      if(!dam.stopBowl())
        nh.logwarn("Cannot stop bowl");
}
ros::Subscriber<std_msgs::Float32> bowl_sub("bowl", bowl_cb);

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

//*** drill target position subscriber ***//
void gotoDrill_cb(const std_msgs::UInt16& cmd_msg){
    if(!dam.gotoDrill(cmd_msg.data)){
      sprintf(buffer, "Cannot set Drill Y axis target to %d", cmd_msg.data);
      nh.logwarn(buffer);
    }
}
ros::Subscriber<std_msgs::UInt16> gotoDrill_sub("drill_y_axis_target", gotoDrill_cb);

//*** probe homing subscriber ***//
void homeProbe_cb(const std_msgs::Empty& home_msg){
  if(!dam.homeProbe())
    nh.logwarn("Couldnt home Probe Y axis");
}
ros::Subscriber<std_msgs::Empty> homeProbe_sub("home_probe", homeProbe_cb);

//*** probe target position subscriber ***//
void gotoProbe_cb(const std_msgs::UInt16& cmd_msg){
    if(!dam.gotoProbe(cmd_msg.data)){
      sprintf(buffer, "Cannot set Probe Y axis target to %d", cmd_msg.data);
      nh.logwarn(buffer);
    }
}
ros::Subscriber<std_msgs::UInt16> gotoProbe_sub("probe_y_axis_target", gotoProbe_cb);

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

//*** probe speed subscriber ***//
void setProbeSpeed_cb(const std_msgs::UInt16& cmd_msg){
    dam.setProbeSpeed(cmd_msg.data);
}
ros::Subscriber<std_msgs::UInt16> setProbeSpeed_sub("set_probe_speed", setProbeSpeed_cb);
//*** drill speed subscriber ***//
void setDrillSpeed_cb(const std_msgs::UInt16& cmd_msg){
    dam.setDrillSpeed(cmd_msg.data);
}
ros::Subscriber<std_msgs::UInt16> setDrillSpeed_sub("set_drill_speed", setDrillSpeed_cb);
//*** x speed subscriber ***//
void setXSpeed_cb(const std_msgs::UInt16& cmd_msg){
    dam.setXSpeed(cmd_msg.data);
}
ros::Subscriber<std_msgs::UInt16> setXSpeed_sub("set_x_speed", setXSpeed_cb);




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
  nh.subscribe(homeX_sub);
  nh.subscribe(homeDrill_sub);
  nh.subscribe(gotoDrill_sub);  
  nh.subscribe(homeProbe_sub);
  nh.subscribe(gotoProbe_sub);
  nh.subscribe(eStop_sub);
  nh.subscribe(resume_sub);
  nh.subscribe(reset_sub);
  nh.subscribe(setProbeSpeed_sub);
  nh.subscribe(setDrillSpeed_sub);
  nh.subscribe(setXSpeed_sub);

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

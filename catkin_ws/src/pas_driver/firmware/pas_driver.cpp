//includes
#include <AccelStepper.h>
#include <Arduino.h>
#include <Pas.h>
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <prismm_msgs/dam_data.h>

#define SEND_RATE 200

ros::NodeHandle nh;
Pas pas(nh);

//******************  Subscribers  *********************//
//*** probe homing subscriber ***//
void homeProbe_cb(const std_msgs::Empty& home_msg){
  if(!pas.homeProbe())
    ROS_WARN("Couldnt home Probe");
}
ros::Subscriber<std_msgs::Empty> homeProbe_sub("home_probe", homeProbe_cb);

//*** probe homing subscriber ***//
void homeY_cb(const std_msgs::Empty& home_msg){
  if(!pas.homeY())
    ROS_WARN("Couldnt home Probe Y axis");
}
ros::Subscriber<std_msgs::Empty> homeY_sub("home_probe_y_axis", homeY_cb);

//*** y target position subscriber ***//
void gotoY_cb(const std_msgs::UInt16& cmd_msg){
    if(!pas.gotoY(cmd_msg.data))
      ROS_WARN("Cannot set Probe Y axis target to %4d", cmd_msg.data);
}
ros::Subscriber<std_msgs::UInt16> gotoY_sub("probe_y_axis_target", gotoY_cb);

//*** probe rotation target position subscriber ***//
void gotoProbeRot_cb(const std_msgs::Float32& cmd_msg){
    if(!pas.gotoProbeRot(cmd_msg.data))
      ROS_WARN("Cannot set Probe rotation target to %4d", cmd_msg.data);
}
ros::Subscriber<std_msgs::Float32> gotoProbeRot_sub("probe_rotation_target", gotoProbeRot_cb);

//*** probe extention target position subscriber ***//
void gotoProbeExt_cb(const std_msgs::Float32& cmd_msg){
    if(!pas.gotoProbeExt(cmd_msg.data))
      ROS_WARN("Cannot set Probe extention target to %4d", cmd_msg.data);
}
ros::Subscriber<std_msgs::Float32> gotoProbeExt_sub("probe_extention_target", gotoProbeExt_cb);

//*** bowl subscriber ***//
void bowl_cb(const std_msgs::Bool& bowl_msg){
    if(bowl_msg.data)
      if(!pas.startBowl())
        ROS_WARN("Cannot start Bowl", cmd_msg.data);
    else
      if(!pas.stopProbe())
        ROS_WARN("Cannot stop Probe", cmd_msg.data);
}
ros::Subscriber<std_msgs::UInt16> bowl_sub("bowl", bowl_cb);

//*** E-Stop subscriber ***//
void eStop_cb(const std_msgs::Empty& e_stop_msg){
  if(!pas.eStop())
    ROS_WARN("Couldnt E-Stop");
}
ros::Subscriber<std_msgs::Empty> eStop_sub("e_stop", eStop_cb);

//*** Resume subscriber ***//
void resume_cb(const std_msgs::Empty& e_stop_msg){
  if(!pas.resume())
    ROS_WARN("Couldnt Resume");
}
ros::Subscriber<std_msgs::Empty> resume_sub("resume", resume_cb);

//*** Reset subscriber ***//
void reset_cb(const std_msgs::Empty& e_stop_msg){
  if(!pas.reset())
    ROS_WARN("Couldnt Reset");
}
ros::Subscriber<std_msgs::Empty> reset_sub("reset", reset_cb);

//*** Pump subscriber ***//
void pump_cb(const std_msgs::Bool& pump_msg){
  if(pump_msg.data)
    if(!pas.enablePump())
      ROS_WARN("Cannot start the Pump", cmd_msg.data);
  else
    if(!pas.dissablePump())
      ROS_WARN("Cannot stop the Pump", cmd_msg.data);
}
ros::Subscriber<std_msgs::Bool> pump_sub("pump", pump_cb);

//******************  Publishers  *********************//
prismm_msgs::pas_data pas_data_msg;
ros::Publisher dpas_data_pub("pas_data", &pas_data_msg);

//*** Vaiables ***//
long last_send = millis();

//*** Setup ***//
void setup() {
  nh.initNode();
  delay(200);

  nh.subscribe(homeProbe_sub);
  nh.subscribe(homeY_sub);
  nh.subscribe(gotoY_sub);
  nh.subscribe(gotoProbeRot_sub);
  nh.subscribe(gotoProbeExt_sub);
  nh.subscribe(bowl_sub);
  nh.subscribe(eStop_sub);
  nh.subscribe(resume_cb);
  nh.subscribe(reset_cb);

  nh.advertise(pas_data_pub);
}


//*** Loop ***//
void loop(){
  nh.spinOnce();
  pas.update();

  //publish dam data
  if(millis()-last_send > SEND_RATE){
    pas_data_msg = pas.getData();
    pas_data_pub.publish(pas_data_msg);
    last_send = millis();
  }
}
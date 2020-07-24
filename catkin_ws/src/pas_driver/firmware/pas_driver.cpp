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
#include <prismm_msgs/getBool.h>

#define SEND_RATE 200

ros::NodeHandle nh;
Pas pas(nh);
char buffer[60];

//******************  Subscribers  *********************//

//*** E-Stop subscriber ***//
void eStop_cb(const std_msgs::Empty& e_stop_msg){
  if(!pas.eStop())
    nh.logwarn("Couldnt E-Stop");
}
ros::Subscriber<std_msgs::Empty> eStop_sub("e_stop", eStop_cb);

//*** Resume subscriber ***//
void resume_cb(const std_msgs::Empty& e_stop_msg){
  if(!pas.resume())
    nh.logwarn("Couldnt Resume");
}
ros::Subscriber<std_msgs::Empty> resume_sub("resume", resume_cb);

//*** Reset subscriber ***//
void reset_cb(const std_msgs::Empty& e_stop_msg){
  if(!pas.reset())
    nh.logwarn("Couldnt Reset");
}
ros::Subscriber<std_msgs::Empty> reset_sub("reset", reset_cb);

//*** Pump subscriber ***//
void pump_cb(const std_msgs::Bool& pump_msg){
  if(pump_msg.data)
    if(!pas.enablePump())
      nh.logwarn("Cannot start the Pump");
  else
    if(!pas.disablePump())
      nh.logwarn("Cannot stop the Pump");
}
ros::Subscriber<std_msgs::Bool> pump_sub("pump", pump_cb);

//******************  Publishers  *********************//
prismm_msgs::pas_data pas_data_msg;
ros::Publisher pas_data_pub("pas_data", &pas_data_msg);


//*** Variables ***//
long last_send = millis();

//*** Setup ***//
void setup() {
  nh.initNode();
  delay(200);

  nh.subscribe(pump_sub);
  nh.subscribe(eStop_sub);
  nh.subscribe(resume_sub);
  nh.subscribe(reset_sub);

  nh.advertise(pas_data_pub);
}


//*** Loop ***//
void loop(){
  nh.spinOnce();
  pas.update();

  //publish dam data
  if(millis()-last_send > SEND_RATE){
    pas_data_msg = pas.getData();
    pas_data_pub.publish(&pas_data_msg);
    last_send = millis();
  }
}
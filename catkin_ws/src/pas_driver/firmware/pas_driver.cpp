//includes
#include <AccelStepper.h>
#include <Arduino.h>
#include <Pas.h>
#include <ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <prismm_msgs/dam_data.h>

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
void pump_cb(const std_msgs::UInt8& pump_msg){
    pas.enablePump(pump_msg.data);
}
ros::Subscriber<std_msgs::UInt8> pump_sub("pump", pump_cb);

//*** Heat subscriber ***//
void heat_cb(const std_msgs::UInt16& heat_msg){
    if(heat_msg.data > 25){
      pas.enableHeater(heat_msg.data);
    } else {
      pas.disableHeater();
    }
}
ros::Subscriber<std_msgs::UInt16> heat_sub("main_heater", heat_cb);

//*** Heat2 subscriber ***//
void heat2_cb(const std_msgs::UInt16& heat_msg){
    if(heat_msg.data > 25){
      pas.enableHeater2(heat_msg.data);
    } else{
      pas.disableHeater2();
    }
}
ros::Subscriber<std_msgs::UInt16> heat2_sub("secondary_heater", heat2_cb);

//*** Power 24v subscriber ***//
void power_cb(const std_msgs::Bool& power_msg){
    if(power_msg.data){
      pas.enablePower();
    } else{
      pas.disablePower();
    }
}
ros::Subscriber<std_msgs::Bool> power_sub("24v_power", power_cb);

//*** Tare subscriber ***//
void tare_cb(const std_msgs::Empty& tare_msg){
    pas.tareLoadCells();
}
ros::Subscriber<std_msgs::Empty> tare_sub("tare_loadcells", tare_cb);

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

  nh.subscribe(power_sub);
  nh.subscribe(heat_sub);
  nh.subscribe(heat2_sub);
  nh.subscribe(tare_sub);

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
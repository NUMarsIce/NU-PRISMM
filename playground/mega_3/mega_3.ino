#include <ros.h>
#include <std_msgs/UInt16.h>
#include <Servo.h>

ros::NodeHandle nh;

Servo rot;
Servo ext;

//*** probe rot target subscriber ***//
void gotoProbeRot_cb(const std_msgs::UInt16& cmd_msg){
    rot.write(cmd_msg.data);
}
ros::Subscriber<std_msgs::UInt16> gotoProbeRot_sub("probe_rot_target", gotoProbeRot_cb);

//*** probe ext position subscriber ***//
void gotoProbeExt_cb(const std_msgs::UInt16& cmd_msg){
    rot.write(cmd_msg.data);
}
ros::Subscriber<std_msgs::UInt16> gotoProbeExt_sub("probe_y_axis_target", gotoProbeExt_cb);


void setup() {
  nh.initNode();
  delay(200);

  nh.subscribe(gotoProbeRot_sub);
  nh.subscribe(gotoProbeExt_sub);

  ext.attach(9);
  rot.attach(10);
}

void loop() {
  nh.spinOnce();
  delay(1);
}

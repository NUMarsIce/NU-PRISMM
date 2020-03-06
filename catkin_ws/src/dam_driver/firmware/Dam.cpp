#include "Dam.h" 

Dam::Dam(ros::NodeHandle nh) : load_cell(LC_DAT_PIN, LC_CLK_PIN),
							stp_y_current_sensor(STP_Y_CURRENT_PIN),
							drill_current_sensor(DRILL_CURRENT_PIN),
							stp_x1(AccelStepper::FULL2WIRE, STP_X1_STEP_PIN, STP_X1_DIR_PIN),
							stp_x2(AccelStepper::FULL2WIRE, STP_X2_STEP_PIN, STP_X2_DIR_PIN),
							stp_y(AccelStepper::FULL2WIRE, STP_Y_STEP_PIN, STP_Y_DIR_PIN){
	//TODO
}

bool Dam::update(){
	//TODO`
}

Dam::DamState Dam::getState(){
	//TODO
}

bool Dam::startDrilling(){
	//TODO
}

bool Dam::stopDrilling(){
	//TODO
}

bool Dam::homeX(){
	//TODO
}

bool Dam::gotoX(int pos){
	//TODO
}

bool Dam::homeDrill(){
	//TODO
}

bool Dam::gotoDrill(int pos){
	//TODO
}

prismm_msgs::dam_data Dam::getData(){
	//TODO
}

bool Dam::eStop(){
	//TODO
}

bool Dam::resume(){
	//TODO
}

bool Dam::reset(){
	//TODO
}


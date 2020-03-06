#include "Pas.h" 

Pas::Pas(ros::NodeHandle nh) : load_cell(LC_DAT_PIN, LC_CLK_PIN),
                                heat_current_sensor(HEAT_CURRENT_PIN),
                                heat2_current_sensor(HEAT2_CURRENT_PIN),
                                stp_rot(AccelStepper::FULL2WIRE, STP_ROT_STEP_PIN, STP_ROT_DIR_PIN),
                                stp_ext(AccelStepper::FULL2WIRE, STP_EXT_STEP_PIN, STP_EXT_DIR_PIN),
                                stp_y(AccelStepper::FULL2WIRE, STP_Y_STEP_PIN, STP_Y_DIR_PIN),
                                heat_therm(HEAT_THERM_PIN, 3950, 10000, 10000),
                                heat2_therm(HEAT2_THERM_PIN, 3950, 10000, 10000) {
	//TODO
}

bool Pas::update(){
	//TODO
}

bool Pas::gotoY(int pos){
	//TODO
}

bool Pas::homeY(){
	//TODO
}

bool Pas::homeProbe(){
	//TODO
}

bool Pas::gotoProbeRot(int angle){
	//TODO
}

bool Pas::gotoProbeExt(int angle){
	//TODO
}

//bool Pas::startRockwell(double max_pressure){
	//TODO
//}

bool Pas::startBowl(double speed){
	//TODO
}

bool Pas::stopProbe(){
	//TODO
}

bool Pas::enableHeater(double max_temp){
	//TODO
}

bool Pas::disableHeater(){
	//TODO
}

bool Pas::enableHeater2(double max_temp){
	//TODO
}

bool Pas::disableHeater2(){
	//TODO
}

bool Pas::enablePump(double speed){
	//TODO
}

bool Pas::disablePump(){
	//TODO
}

prismm_msgs::pas_data Pas::getData(){
	//TODO
}

Pas::PasState Pas::getState(){
	//TODO
}

bool Pas::eStop(){
	//TODO
}

bool Pas::resume(){
	//TODO
}

bool Pas::reset(){
	//TODO
}


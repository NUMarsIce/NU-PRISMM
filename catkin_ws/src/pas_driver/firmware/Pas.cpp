#include "Pas.h" 

Pas::Pas(ros::NodeHandle nh) :  load_cell(LC_DAT_PIN, LC_CLK_PIN),
                                heat_current_sensor(HEAT_CURRENT_PIN),
                                heat2_current_sensor(HEAT2_CURRENT_PIN),
                                stp_rot(AccelStepper::FULL2WIRE, STP_ROT_STEP_PIN, STP_ROT_DIR_PIN),
                                stp_ext(AccelStepper::FULL2WIRE, STP_EXT_STEP_PIN, STP_EXT_DIR_PIN),
                                stp_y(AccelStepper::FULL2WIRE, STP_Y_STEP_PIN, STP_Y_DIR_PIN),
                                heat_therm(HEAT_THERM_PIN, 3950, 10000, 10000),
                                heat2_therm(HEAT2_THERM_PIN, 3950, 10000, 10000) {

    stp_rot.setMaxSpeed(rot_max_speed);
    stp_rot.setAcceleration(STP_ACCEL);
    stp_ext.setMaxSpeed(ext_max_speed);
    stp_ext.setAcceleration(STP_ACCEL);
    stp_y.setMaxSpeed(y_max_speed);
    stp_y.setAcceleration(STP_ACCEL);
}

bool Pas::update(){
	//check if estop has been triggered
	switch(state){
		case E_STOP:
			delay(10); //idle
			return false;
			break;
		case HOMING:
			incrementYHome();
			break;
		case HOMING_PROBE:
			incrementProbeHome();			
			break;
		default:
			stp_rot.run();
			stp_ext.run();
			stp_y.run();
			break;
	}
	return true;
}

bool Pas::gotoY(int pos){
	if(pos < 0 || !probeIsHomed())
		return false;

	stp_y.moveTo(pos*y_step_per_mm);
	return true;
}

bool Pas::gotoProbeRot(int angle){
	if(angle < 0)
		return false;

	stp_rot.moveTo(angle*rot_step_per_degree);
	return true;
}

bool Pas::gotoProbeExt(int angle){
	if(angle < 0)
		return false;

	stp_ext.moveTo(angle*ext_step_per_degree);
	return true;
}

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
	data_out.state = state;
	data_out.stp_ext = (float)stp_ext.currentPosition()/ext_step_per_degree;
	data_out.stp_rot = (float)stp_rot.currentPosition()/rot_step_per_degree;
	data_out.stp_y = (float)stp_y.currentPosition()/y_step_per_mm;

	data_out.heat_temp = 0;//TODO
	data_out.heat_current = 0;//TODO
	data_out.heat2_temp = 0;//TODO
	data_out.heat2_current = 0;//TODO
	data_out.load = 0;//TODO

	data_out.stamp = nh.now();
	return data_out;
}

Pas::PasState Pas::getState(){
	return state;
}

bool Pas::eStop(){
	last_state = state;
	state = E_STOP; 
	return true;
}

bool Pas::resume(){
	if(state != E_STOP)
		return false;
	state = last_state;
	last_state = E_STOP;
	return true;
}

bool Pas::reset(){
	stp_ext.stop();
	stp_rot.stop();
	stp_y.stop();
	if(state != E_STOP)
		return false;
	state = DEFAULT_STATE;
	last_state = E_STOP;
	return true;
}

bool Pas::probeIsHomed(){
	return digitalRead(STP_EXT_HOME_PIN) && digitalRead(STP_ROT_HOME_PIN);
}

bool Pas::homeY(){
	state = HOMING;
	stp_y.setMaxSpeed(y_home_speed);
	stp_y.moveTo(0);
}

bool Pas::homeProbe(){
	state = HOMING_PROBE;
	stp_ext.setMaxSpeed(y_home_speed);
	stp_rot.setMaxSpeed(y_home_speed);
	stp_ext.moveTo(0);
	stp_rot.moveTo(0);
}

void Pas::incrementProbeHome(){
	if(digitalRead(STP_EXT_HOME_PIN)==LOW){
		if(stp_ext.distanceToGo() != 0)
			stp_ext.run();
		else 
			stp_ext.move(-4);
	}
	if(digitalRead(STP_ROT_HOME_PIN)==LOW){
		if(stp_rot.distanceToGo() != 0)
			stp_rot.run();
		else 
			stp_rot.move(-4);
	}
	if(probeIsHomed() && state == HOMING_PROBE){
		stp_ext.stop();
		stp_ext.setCurrentPosition(0);
		stp_rot.stop();
		stp_rot.setCurrentPosition(0);

		stp_ext.setMaxSpeed(ext_max_speed);
		stp_rot.setMaxSpeed(rot_max_speed);
		state = DEFAULT_STATE;
	}
}

void Pas::incrementYHome(){
	if(!probeIsHomed()){
		incrementProbeHome();
		return;
	}

	if(digitalRead(STP_Y_HOME_PIN)==LOW){
		if(stp_y.distanceToGo() != 0)
			stp_y.run();
		else 
			stp_y.move(-4);
	} else {
		stp_y.stop();
		stp_y.setCurrentPosition(0);
		stp_y.setMaxSpeed(y_max_speed);
		state = DEFAULT_STATE;
	}
}
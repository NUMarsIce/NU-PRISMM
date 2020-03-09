#include "Dam.h" 

Dam::Dam(ros::NodeHandle nh) :  load_cell(LC_DAT_PIN, LC_CLK_PIN),
								stp_y_current_sensor(STP_Y_CURRENT_PIN),
								drill_current_sensor(DRILL_CURRENT_PIN),
								stp_x(AccelStepper::FULL2WIRE, STP_X_STEP_PIN, STP_X_DIR_PIN),
								stp_y(AccelStepper::FULL2WIRE, STP_Y_STEP_PIN, STP_Y_DIR_PIN),
								probe_homed("probe_is_homed"){
    stp_x.setMaxSpeed(x_max_speed);
    stp_x.setAcceleration(STP_ACCEL);
    stp_y.setMaxSpeed(y_max_speed);
    stp_y.setAcceleration(STP_ACCEL);
	
	nh.serviceClient(probe_homed);
}

bool Dam::update(){
	//check if estop has been triggered
	switch(state){
		case E_STOP:
			delay(10); //idle
			return false;
			break;
		case HOMING:
			incrementXHome();
			break;
		case HOMING_DRILL:
			incrementYHome();			
			break;
		default:
			stp_y.run();
			stp_x.run();
			break;
	}
	return true;}

bool Dam::startDrilling(){
	//TODO
}

bool Dam::stopDrilling(){
	//TODO
}

bool Dam::homeX(){
	if(probeNotHomed() && false)
		return false;

	state = HOMING;
	stp_x.setMaxSpeed(x_home_speed);
	stp_x.moveTo(0);
}

bool Dam::gotoX(int pos){
	if(pos < 0 )//|| probeNotHomed()
		return false;

	stp_x.moveTo(pos*x_step_per_mm);
	return true;
}

bool Dam::homeDrill(){
	state = HOMING_DRILL;
	stp_y.setMaxSpeed(y_home_speed);
	stp_y.moveTo(0);
}

bool Dam::gotoDrill(int pos){
	if(pos < 0)
		return false;

	stp_y.moveTo(pos*y_step_per_mm);
	return true;
}

prismm_msgs::dam_data Dam::getData(){
	data_out.state = state;
	data_out.stp_x1 = (float)stp_x.currentPosition()/x_step_per_mm;
	data_out.stp_x2 = 0;
	data_out.stp_y = (float)stp_y.currentPosition()/y_step_per_mm;

	data_out.drill_stp_current = 0;//TODO
	data_out.drill_current = 0;//TODO
	data_out.load = 0;//TODO

	data_out.stamp = nh.now();
	return data_out;
}

Dam::DamState Dam::getState(){
	return state;
}

bool Dam::eStop(){
	last_state = state;
	state = E_STOP; 
	return true;
}

bool Dam::resume(){
	if(state != E_STOP)
		return false;
	state = last_state;
	last_state = E_STOP;
	return true;
}

bool Dam::reset(){
	stp_x.stop();
	stp_y.stop();
	if(state != E_STOP)
		return false;
	state = DEFAULT_STATE;
	last_state = E_STOP;
	return true;
}

bool Dam::probeNotHomed(){
	probe_homed.call(probe_srv_req, probe_srv_resp);//TODO crashes for some reason
	return !probe_srv_resp.data;
}

void Dam::incrementYHome(){
	if(digitalRead(STP_Y_HOME_PIN)==LOW){
		if(stp_y.distanceToGo() != 0)
			stp_y.run();
		else 
			stp_y.move(-4);
	} else {
		stp_y.stop();
		stp_y.setCurrentPosition(0);
		stp_y.setMaxSpeed(y_max_speed);
		if(state == HOMING_DRILL)
			state = DEFAULT_STATE;
	}
}

void Dam::incrementXHome(){
	if(false && probeNotHomed()){//Cant home if probe not homed
		state = last_state;
		return;
	}else if(digitalRead(STP_Y_HOME_PIN) == LOW && false){
		incrementYHome();
		return;
	}

	if(digitalRead(STP_X_HOME_PIN) == LOW){
		if(stp_x.distanceToGo() != 0)
			stp_x.run();
		else 
			stp_x.move(-4);
	} else {
		stp_x.stop();
		stp_x.setCurrentPosition(0);
		stp_x.setMaxSpeed(y_max_speed);
		state = DEFAULT_STATE;
	}
}
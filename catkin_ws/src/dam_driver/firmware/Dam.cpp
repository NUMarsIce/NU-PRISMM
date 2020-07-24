#include "Dam.h" 

Dam::Dam(ros::NodeHandle nh):stp_drill_current_sensor(STP_DRILL_CURRENT_PIN, 100),
							drill_current_avg(100),
							stp_x(AccelStepper::FULL2WIRE, STP_X_STEP_PIN, STP_X_DIR_PIN),
							stp_drill(AccelStepper::FULL2WIRE, STP_DRILL_STEP_PIN, STP_DRILL_DIR_PIN),
							stp_probe(AccelStepper::FULL2WIRE, STP_PROBE_STEP_PIN, STP_PROBE_DIR_PIN){
    stp_x.setMaxSpeed(x_max_speed);
    stp_x.setAcceleration(STP_ACCEL);
    stp_drill.setMaxSpeed(drill_max_speed);
    stp_drill.setAcceleration(STP_ACCEL);
    stp_probe.setMaxSpeed(probe_max_speed);
    stp_probe.setAcceleration(STP_ACCEL);
	
	pinMode(STP_DRILL_HOME_PIN, OUTPUT);
	pinMode(STP_PROBE_HOME_PIN, OUTPUT);
	pinMode(STP_X_HOME_PIN, OUTPUT);

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
			incrementDrillHome();			
			break;
		case HOMING_PROBE:
			incrementProbeHome();			
			break;
		case BOWL:
			iterateBowl();
			break;
		case ROCKWELL:
			iterateRockwell();
			break;
		default:
			stp_drill.run();
			stp_probe.run();
			stp_x.run();
			break;
	}
	return true;}


bool Dam::gotoProbeRot(int angle){
	if(angle < 0)
		return false;

	servo_rot.write(angle);
	return true;
}

bool Dam::gotoProbeExt(int angle){
	if(angle < 0)
		return false;

	servo_ext.write(angle);
	return true;
}

bool Dam::startBowl(double speed){
	//TODO publisher for relays
}

bool Dam::stopBowl(){
	//TODO
}

void iterateBowl(){
	//TODO 
}

void iterateRockwell(){
	//TODO
}

bool Dam::startDrilling(){
	//TODO
}

bool Dam::stopDrilling(){
	//TODO
}

bool Dam::homeX(){
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
	stp_drill.setMaxSpeed(drill_home_speed);
	stp_drill.moveTo(0);
}

bool Dam::gotoDrill(int pos){
	if(pos < 0)
		return false;

	stp_drill.moveTo(pos*drill_step_per_mm);
	return true;
}

bool Dam::homeProbe(){
	servo_ext.write(0);
	servo_rot.write(0);
}

bool Dam::gotoProbe(int pos){
	if(pos < 0)
		return false;

	stp_probe.moveTo(pos*probe_step_per_mm);
	return true;
}

prismm_msgs::dam_data Dam::getData(){
	data_out.state = state;
	data_out.stp_drill = (float)stp_drill.currentPosition()/drill_step_per_mm;
	data_out.stp_probe = (float)stp_probe.currentPosition()/probe_step_per_mm;
	data_out.stp_x = (float)stp_x.currentPosition()/x_step_per_mm;

	data_out.servo_ext = servo_ext.read();
	data_out.servo_rot = servo_rot.read();

	data_out.drill_stp_current = drill_current_avg.process(stp_drill_current_sensor.read());

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
	stp_drill.stop();
	stp_probe.stop();
	if(state != E_STOP)
		return false;
	state = DEFAULT_STATE;
	last_state = E_STOP;
	return true;
}

bool Dam::probeNotHomed(){
	return servo_ext.read() != 0 || servo_rot.read() != 0;
}

void Dam::incrementDrillHome(){
	if(!digitalRead(STP_DRILL_HOME_PIN)){
		if(stp_drill.distanceToGo() != 0)
			stp_drill.run();
		else 
			stp_drill.move(-4);
	} else {
		stp_drill.stop();
		stp_drill.setCurrentPosition(0);
		stp_drill.setMaxSpeed(drill_max_speed);
		if(state == HOMING_DRILL)
			state = DEFAULT_STATE;
	}
}

void Dam::incrementProbeHome(){
	if(probeNotHomed()){
		servo_ext.write(0);
		servo_rot.write(0);
	}

	if(!digitalRead(STP_PROBE_HOME_PIN)){
		if(stp_probe.distanceToGo() != 0)
			stp_probe.run();
		else 
			stp_probe.move(-4);
	} else {
		stp_probe.stop();
		stp_probe.setCurrentPosition(0);
		stp_probe.setMaxSpeed(probe_max_speed);
		if(state == HOMING_PROBE)
			state = DEFAULT_STATE;
	}
}

void Dam::incrementXHome(){
	if(probeNotHomed()){//Cant home if probe not homed
		state = last_state;
		return;
	}else if(!digitalRead(STP_DRILL_HOME_PIN) || !digitalRead(STP_PROBE_HOME_PIN)){
		incrementDrillHome();
		incrementProbeHome();
		return;
	}

	if(!digitalRead(STP_X_HOME_PIN)){
		if(stp_x.distanceToGo() != 0)
			stp_x.run();
		else 
			stp_x.move(-4);
	} else {
		stp_x.stop();
		stp_x.setCurrentPosition(0);
		stp_x.setMaxSpeed(x_max_speed);
		state = DEFAULT_STATE;
	}
}
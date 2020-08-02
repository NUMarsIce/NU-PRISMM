#include "Pas.h" 

Pas::Pas(ros::NodeHandle nh) :  load_cell_A(LCA_DAT_PIN, LCA_CLK_PIN),
								load_cell_B(LCB_DAT_PIN, LCB_CLK_PIN),
								heat_current_avg(10),
								drill_current_avg(10),
								power_current_avg(10),
								pow5_current_avg(10),
								pow24_current_avg(100),
                                heat_current_sensor(HEAT_CURRENT_PIN, 100),
                                drill_current_sensor(DRILL_CURRENT_PIN, 100),
                                power_current_sensor(POWER_CURRENT_PIN),
                                pow24_current_sensor(POW24_CURRENT_PIN),
                                pow5_current_sensor(POW5_CURRENT_PIN, 100),
                                heat_therm(HEAT_THERM_PIN),
                                heat2_therm(HEAT2_THERM_PIN),
                                ambient_therm(AMBIENT_THERM_PIN, 3950, 10000, 10000) {

	pinMode(E_STOP_PIN, INPUT_PULLUP);
	pinMode(HEAT2_RELAY_PIN, OUTPUT);
	pinMode(DRILL_RELAY_PIN, OUTPUT);
	pinMode(HEAT_RELAY_PIN, OUTPUT);
	pinMode(POWER_RELAY_PIN, OUTPUT);
	pinMode(PUMP_SPEED_PIN, OUTPUT);
	pinMode(PUMP_DIR_PIN, OUTPUT);
	
	load_cell_A.begin(LCA_DAT_PIN, LCA_CLK_PIN);
	load_cell_A.set_scale(LOADA_CAL_FACTOR); 
	load_cell_B.begin(LCB_DAT_PIN, LCB_CLK_PIN);
	load_cell_B.set_scale(LOADB_CAL_FACTOR); 
 	load_cell_A.tare(); 
        load_cell_B.tare(); 
	
	this->nh = nh;

}

bool Pas::update(){
	//check if estop has been triggered
	if(digitalRead(E_STOP_PIN) == LOW)
		state = E_STOP;
	switch(state){
		case E_STOP:
			delay(10); //idle
			return false;
			break;
		default:
                heat_current_sensor.read();
        	drill_current_sensor.read();
        	power_current_sensor.read();
        	pow24_current_sensor.read();
        	pow5_current_sensor.read();
			//heating "control loop"
			if(heat_therm.read() < heat_target-10 && heat_target != 0)
				digitalWrite(HEAT_RELAY_PIN, HIGH);
			else
				digitalWrite(HEAT_RELAY_PIN, LOW);
			if(heat2_therm.read() < heat2_target-10 && heat2_target != 0)
				digitalWrite(HEAT2_RELAY_PIN, HIGH);
			else
				digitalWrite(HEAT2_RELAY_PIN, LOW);
			break;
	}
	return true;
}

void Pas::tareLoadCells(){
	load_cell_A.tare(); 
    load_cell_B.tare(); 
}

bool Pas::enableHeater(double max_temp){
	heat_target = max_temp;
}

bool Pas::disableHeater(){
	heat_target = 0;
}

bool Pas::enableDrill(){
	digitalWrite(DRILL_RELAY_PIN, HIGH);
}

bool Pas::disableDrill(){
	digitalWrite(DRILL_RELAY_PIN, LOW);
}

bool Pas::enableHeater2(double max_temp){
	heat2_target = max_temp;
}

bool Pas::disableHeater2(){
	heat2_target = 0;
}

bool Pas::enablePower(){
	digitalWrite(POWER_RELAY_PIN, HIGH);
}

bool Pas::disablePower(){
	digitalWrite(POWER_RELAY_PIN, LOW);
}

bool Pas::enablePump(double speed){
	analogWrite(PUMP_SPEED_PIN, (int)(abs(speed)*255));
	digitalWrite(PUMP_DIR_PIN, abs(speed) == speed);
}

bool Pas::disablePump(){
	analogWrite(PUMP_SPEED_PIN, 0);
}

bool Pas::setFilterState(FilterState state){
	filter_state = state;
	switch(state){
		case OFF:
		default:
			enablePump(0);
			digitalWrite(OUTPUT_RELAY_PIN, LOW);
			digitalWrite(BYPASS_RELAY_PIN, LOW);
			digitalWrite(RECIRCULATE_RELAY_PIN, LOW);
			digitalWrite(COARSE_YEET_RELAY_PIN, LOW);
			digitalWrite(MEDIUM_YEET_RELAY_PIN, LOW);
		break;
		case NORMAL_FILTER:
			enablePump();
			digitalWrite(OUTPUT_RELAY_PIN, HIGH);
			digitalWrite(BYPASS_RELAY_PIN, LOW);
			digitalWrite(RECIRCULATE_RELAY_PIN, LOW);
			digitalWrite(COARSE_YEET_RELAY_PIN, LOW);
			digitalWrite(MEDIUM_YEET_RELAY_PIN, LOW);
		break;
		case BYPASS:
			enablePump();
			digitalWrite(OUTPUT_RELAY_PIN, LOW);
			digitalWrite(BYPASS_RELAY_PIN, HIGH);
			digitalWrite(RECIRCULATE_RELAY_PIN, LOW);
			digitalWrite(COARSE_YEET_RELAY_PIN, HIGH);
			digitalWrite(MEDIUM_YEET_RELAY_PIN, HIGH);
		break;
		case RECIRCULATE:
			enablePump();
			digitalWrite(OUTPUT_RELAY_PIN, LOW);
			digitalWrite(BYPASS_RELAY_PIN, LOW);
			digitalWrite(RECIRCULATE_RELAY_PIN, HIGH);
			digitalWrite(COARSE_YEET_RELAY_PIN, HIGH);
			digitalWrite(MEDIUM_YEET_RELAY_PIN, HIGH);
		break;
	}
}

prismm_msgs::pas_data Pas::getData(){
	data_out.state = state;
	data_out.filter_state = filter_state;

	data_out.heat_current = heat_current_avg.process(heat_current_sensor.read());
	data_out.drill_current = drill_current_avg.process(drill_current_sensor.read());
	data_out.power_current = power_current_avg.process(power_current_sensor.read());
	data_out.pow24_current = pow24_current_avg.process(pow24_current_sensor.read());
	data_out.pow5_current = pow5_current_avg.process(pow5_current_sensor.read());
	
	data_out.heat_temp = heat_therm.read();
	data_out.heat2_temp = heat2_therm.read();
	data_out.ambient_temp = ambient_therm.read();

	data_out.loadA = load_cell_A.get_units();
	data_out.loadB = load_cell_B.get_units();

	data_out.heater = digitalRead(HEAT_RELAY_PIN);
	data_out.heater2 = digitalRead(HEAT2_RELAY_PIN);
	data_out.power24 = digitalRead(POWER_RELAY_PIN);
	data_out.drill = digitalRead(DRILL_RELAY_PIN);

	data_out.drill_speed = Serial1.parseInt();
	while (Serial1.available())
		Serial1.read();

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
	if(state != E_STOP)
		return false;
	state = DEFAULT_STATE;
	last_state = E_STOP;
	return true;
}



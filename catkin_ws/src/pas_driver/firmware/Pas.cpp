#include "Pas.h" 

Pas::Pas(ros::NodeHandle nh) :  load_cell_A(LCA_DAT_PIN, LCA_CLK_PIN),
								load_cell_B(LCB_DAT_PIN, LCB_CLK_PIN),
								heat_current_avg(100),
								drill_current_avg(100),
								power_current_avg(100),
								pow5_current_avg(100),
								pow24_current_avg(100),
                                heat_current_sensor(HEAT_CURRENT_PIN, 100),
                                drill_current_sensor(DRILL_CURRENT_PIN, 100),
                                power_current_sensor(POWER_CURRENT_PIN),
                                pow24_current_sensor(POW24_CURRENT_PIN),
                                pow5_current_sensor(POW5_CURRENT_PIN),
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
	
	self.nh = nh;

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
			break;
	}
	return true;
}

void tareLoadCells(){
	load_cell_A.tare(); 
    load_cell_B.tare(); 
}

bool Pas::enableHeater(double max_temp){
	digitalWrite(HEAT_RELAY_PIN, HIGH);
}

bool Pas::disableHeater(){
	digitalWrite(HEAT_RELAY_PIN, LOW);
}

bool Pas::enableHeater2(double max_temp){
	digitalWrite(HEAT2_RELAY_PIN, HIGH);
}

bool Pas::disableHeater2(){
	digitalWrite(HEAT2_RELAY_PIN, LOW);
}

bool Pas::enablePower(){
	digitalWrite(POWER_RELAY_PIN, HIGH);
}

bool Pas::disablePower(){
	digitalWrite(POWER_RELAY_PIN, LOW);
}

bool Pas::enablePump(double speed){
	analogWrite(PUMP_SPEED_PIN, (int)(speed*255));
}

bool Pas::disablePump(){
	analogWrite(PUMP_SPEED_PIN, 0);
}

prismm_msgs::pas_data Pas::getData(){
	data_out.state = state;

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



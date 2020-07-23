#ifndef _SENSE_H_
#define _SENSE_H_

//includes
#include <ACS712.h>
#include <ACS712_AC.h>
#include <AccelStepper.h>
#include <Thermistor.h>
#include <HX711.h>
#include <ros.h>
#include <prismm_msgs/pas_data.h>
#include <MovingAverageFilter.h>
#include <Servo.h>

// 
#define STP_ACCEL 2000
#define STEPS_PER_REV 800

// DC pump motor
#define PUMP_DIR_PIN 3//TODO
#define PUMP_SPEED_PIN 5

// Relays
#define HEAT_RELAY_PIN 1
#define HEAT2_RELAY_PIN 2
#define POWER_RELAY_PIN 14
#define DRILL_RELAY_PIN 0

//Current Sensor
#define HEAT_CURRENT_PIN A9
#define DRILL_CURRENT_PIN A8
#define POWER_CURRENT_PIN A10
#define POW24_CURRENT_PIN A6
#define POW5_CURRENT_PIN A5

//Thermistors
#define HEAT_THERM_PIN A15
#define HEAT2_THERM_PIN A14
#define AMBIENT_THERM_PIN A13

// Load Cell
#define LCA_DAT_PIN 10
#define LCA_CLK_PIN 11
#define LCB_DAT_PIN 12
#define LCB_CLK_PIN 13

//Switches
#define E_STOP_PIN 3//TODO

class Pas {
    public:
        Pas(ros::NodeHandle nh);

    enum PasState{
        E_STOP = -1,
        DEFAULT_STATE = 0,
    };

    bool update();

    bool enableHeater(double max_temp);
    bool disableHeater();
    bool enableHeater2(double max_temp);
    bool disableHeater2();

    bool enablePump(double speed = 1.0);
    bool disablePump();

    PasState getState();
    prismm_msgs::pas_data getData();

    bool eStop();
    bool resume();// Continue trilling or moving with motors or heating after e stop
    bool reset();// Resume processing but reset motor movements, state, and any heating

    private:
        ros::NodeHandle nh;

        bool heat_on = true;
        bool heat2_on = false;
        bool e_stopped = false;
        PasState last_state = DEFAULT_STATE;
        PasState state = DEFAULT_STATE;
        prismm_msgs::pas_data data_out;

        HX711 load_cell_A;
        HX711 load_cell_B;
        ACS712_AC heat_current_sensor;
        ACS712_AC drill_current_sensor;
        ACS712_AC power_current_sensor;
        ACS712_AC pow12_current_sensor;
        ACS712_AC pow5_current_sensor;

        Servo servo_ext;
        Servo serbo_rot;

        Thermistor heat_therm;
        Thermistor heat2_therm;
        Thermistor ambient_therm;

        MovingAverageFilter heat_current_avg(100);
        MovingAverageFilter drill_current_avg(100);
        MovingAverageFilter power_current_avg(100);
        MovingAverageFilter pow24_current_avg(100);
        MovingAverageFilter pow5_current_avg(100);


		void interateBowl();

};

#endif /** _Dam_H_ **/

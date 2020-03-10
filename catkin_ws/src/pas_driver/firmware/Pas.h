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

// 
#define STP_ACCEL 2000
#define STEPS_PER_REV 800
// DC pump motor
#define PUMP_DIR_PIN 15
#define PUMP_SPEED_PIN 16

// Y Stepper
#define STP_Y_STEP_PIN 6
#define STP_Y_DIR_PIN 5
#define STP_Y_HOME_PIN 4 
 
// Probe Rotation Stepper
#define STP_ROT_STEP_PIN 10
#define STP_ROT_DIR_PIN 9
#define STP_ROT_HOME_PIN 8

// Probe Extention Stepper
#define STP_EXT_STEP_PIN 14
#define STP_EXT_DIR_PIN 13
#define STP_EXT_HOME_PIN 12

// Relays
#define HEAT_RELAY_PIN 18
#define HEAT2_RELAY_PIN 19

//Current Sensor
#define HEAT_CURRENT_PIN 21
#define HEAT2_CURRENT_PIN 22

//Thermistors
#define HEAT_THERM_PIN 24
#define HEAT2_THERM_PIN 25

// Load Cell
#define LC_DAT_PIN 27
#define LC_CLK_PIN 28

//Switches
#define E_STOP_PIN 30

class Pas {
    public:
        Pas(ros::NodeHandle nh);

    enum PasState{
        E_STOP = -1,
        DEFAULT_STATE = 0,
        HOMED = 1,
        HOMING_PROBE = 2,
        HOMING = 3,
        BOWL = 4,
        ROCKWELL = 5,
    };

    bool update();

    bool gotoY(int pos); // Return false if distance is out of bounds
    bool homeY();// Return false if probe not homed
    bool homeProbe();
    bool gotoProbeRot(int angle);
    bool gotoProbeExt(int angle);

    //bool startRockwell(double max_pressure);// Press probe down and melt ice (or just heat)
    bool startBowl(double speed = 1.0);// Return false if homed (or we know we aren't near ice)
    bool stopProbe();

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

        float y_step_per_mm = 800;
        float rot_step_per_degree = 800;
        float ext_step_per_degree = 800;

        float y_home_speed = 200.0;//currently in steps per second
        float ext_home_speed = 200.0;
        float rot_home_speed = 200.0;

        float y_max_speed = 2000.0;//currently in steps per second
        float ext_max_speed = 2000.0;
        float rot_max_speed = 2000.0;


        HX711 load_cell;
        ACS712_AC heat_current_sensor;
        ACS712_AC heat2_current_sensor;

        AccelStepper stp_rot;
        AccelStepper stp_ext;
        AccelStepper stp_y;

        Thermistor heat_therm;
        Thermistor heat2_therm;

        bool probeIsHomed();
        void incrementProbeHome();
        void incrementYHome();
};

#endif /** _Dam_H_ **/

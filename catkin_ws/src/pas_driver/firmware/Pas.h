#ifndef _SENSE_H_
#define _SENSE_H_

//includes
#include <ACS712.h>
#include <ACS712_AC.h>
#include <AccelStepper.h>
#include <Thermistor.h>
#include <prismm_msgs/pas_data.h>

// 
#define RAPID_STP_SPEED 1000
#define HOME_STP_SPEED 1000
#define STEPS_PER_REV 800

// DC pump motor
#define PUMP_DIR_PIN 20
#define PUMP_SPEED_PIN 21

// Y Stepper
#define STP_Y_STEP_PIN 6
#define STP_Y_DIR_PIN 5
#define STP_Y_HOME_PIN 77 
 
// Probe Rotation Stepper
#define STP_ROT_STEP_PIN 8
#define STP_ROT_DIR_PIN 7
#define STP_ROT_HOME_PIN 78 

// Probe Extention Stepper
#define STP_EXT_STEP_PIN 10
#define STP_EXT_DIR_PIN 9
#define STP_EXT_HOME_PIN 79 

// Relays
#define HEAT_RELAY_PIN 20
#define HEAT2_RELAY_PIN 20

//Current Sensor
#define HEAT_CURRENT_PIN 20
#define HEAT2_CURRENT_PIN 20

//Thermistors
#define HEAT_THERM_PIN 25
#define HEAT2_THERM_PIN 25

// Load Cell
#define LC_DAT_PIN 26
#define LC_CLK_PIN 27

//Switches
#define E_STOP_PIN 20

class Pas {
    public:
        Pas(ros::NodeHandle nh);
        ~Pas();

    enum PasState{
        E_STOP = -1,
        DEFAULT_STATE = 0,
        HOMED = 1,
        HOMING_PROBE = 2,
        HOMING = 3,
        BOWL = 4,
        ROCKWELL = 5,
    };

    bool gotoY(int dist, int speed); // Return false if distance is out of bounds
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

    prismm_msgs::pas_data getData();

    void eStop();
    void resume();// Continue trilling or moving with motors or heating after e stop
    void reset();// Resume processing but reset motor movements, state, and any heating

    private:
        ros::NodeHandle nh;

        bool heat_on = true;
        bool heat2_on = false;
        bool e_stopped = false;
        bool last_state = PasState.DEFAULT_STATE;
        PasState state = 0;

        HX711 load_cell;
        ACS712_AC heat_current_sensor;
        ACS712_AC heat2_current_sensor;

        AccelStepper stp_rot;
        AccelStepper stp_ext;
        AccelStepper stp_y;

        Thermistor heat_therm;
        Thermistor heat2_therm;

}

#endif /** _Dam_H_ **/

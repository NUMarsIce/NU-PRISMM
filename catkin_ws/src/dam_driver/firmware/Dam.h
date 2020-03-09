#ifndef _DAM_H_
#define _DAM_H_

//includes
#include <ACS712.h>
#include <ACS712_AC.h>
#include <AccelStepper.h>
#include <HX711.h>
#include <ros.h>
#include <prismm_msgs/dam_data.h>
#include <prismm_msgs/getBool.h>

// 
#define STP_ACCEL 2000
#define STEPS_PER_REV 800

// X Steppers
#define STP_X_STEP_PIN 6
#define STP_X_DIR_PIN 5
#define STP_X_HOME_PIN 4  

// Y Stepper
#define STP_Y_STEP_PIN 10
#define STP_Y_DIR_PIN 9
#define STP_Y_HOME_PIN 79
#define STP_Y_CURRENT_PIN A0

// Relays
#define DRILL_RELAY_PIN 20

// Current Sensor
#define DRILL_CURRENT_PIN 20

// Hall effect
#define DRILL_HALL_PIN 20

// Load Cell
#define LC_DAT_PIN 30
#define LC_CLK_PIN 29

//Switches
#define E_STOP_PIN 20

/**
 * 
 * Class that represents the drill and movement for PRISMM
 * 
 * */

class Dam {
    public:
        Dam(ros::NodeHandle nh);

        enum DamState{
            E_STOP = -1,
            DEFAULT_STATE = 0,
            HOMED = 1,
            HOMING = 2,
            HOMING_DRILL = 3,
            DRILLING = 4,
        };

        bool update();// Should be run in loop and do iterative processes
        DamState getState();
        bool startDrilling();// Return false if probe not homed
        bool stopDrilling();

        bool homeX();// Return false if drill and probe not homed
        bool gotoX(int pos); // Return false if distance is out of bounds

        bool homeDrill(); 
        bool gotoDrill(int pos); // Return false if distance is out of bounds

        prismm_msgs::dam_data getData();

        bool eStop();
        bool resume();// Continue drilling or moving with motors after e stop
        bool reset();// Resume processing but reset motor movements and state

        bool probeNotHomed();

    private:
        ros::NodeHandle nh;
        ros::ServiceClient<prismm_msgs::getBoolRequest, prismm_msgs::getBoolResponse> probe_homed;
        const prismm_msgs::getBoolRequest probe_srv_req;
        prismm_msgs::getBoolResponse probe_srv_resp;

        bool tool_is_drill = true;
        bool drill_is_on = false;
        bool e_stopped = false;
        DamState last_state = DEFAULT_STATE;
        DamState state = DEFAULT_STATE;
        prismm_msgs::dam_data data_out;

        int y_step_per_mm = 800;
        int x_step_per_mm = 800;

        float y_home_speed = 200.0;//currently in steps per second
        float x_home_speed = 200.0;

        float y_max_speed = 2000.0;//currently in steps per second
        float x_max_speed = 2000.0;

        HX711 load_cell;
        ACS712 stp_y_current_sensor;
        ACS712_AC drill_current_sensor;

        AccelStepper stp_x;
        AccelStepper stp_y;

        void incrementYHome();
        void incrementXHome();
};

#endif /** _Dam_H_ **/

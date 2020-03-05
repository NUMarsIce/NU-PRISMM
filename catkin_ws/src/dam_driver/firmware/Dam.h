#ifndef _DAM_H_
#define _DAM_H_

//includes
#include <ACS712.h>
#include <ACS712_AC.h>
#include <AccelStepper.h>
#include <HX711.h>
#include <prismm_msgs/dam_data.h>

// 
#define RAPID_STP_SPEED 1000
#define DRILL_STP_SPEED 100

#define TOOL_DISTANCE 100

// X Steppers
#define STP_X1_STEP_PIN 6
#define STP_X1_DIR_PIN 5
#define STP_X1_HOME_PIN 77  
#define STP_X2_STEP_PIN 8
#define STP_X2_DIR_PIN 7
#define STP_X2_HOME_PIN 78  

// Y Stepper
#define STP_Y_STEP_PIN 10
#define STP_Y_DIR_PIN 9
#define STP_Y_HOME_PIN 79
#define STP_Y_CURRENT_PIN A0

// Relays
#define DRILL_RELAY_PIN 20

// Hall effect
#define DRILL_HALL_PIN 20

// Load Cell
#define LC_DAT_PIN 30
#define LC_CLK_PIN 29



/**
 * 
 * Class that represents the drill and movement for PRISMM
 * 
 * */

class Dam {
    public:
        Dam(ros::NodeHandle nh);
        ~Dam();

        enum DamState{
            E_STOP = -1,
            DEFAULT_STATE = 0,
            HOMED = 1,
            HOMING = 2,
            DRILLING = 3,
        };

        bool update();// Should be run in loop and do iterative processes
        DamState getState();
        bool startDrilling(int speed = DRILL_STP_SPEED);// Return false if probe not homed

        bool homeX();// Return false if drill and probe not homed
        bool gotoX(int dist, int speed); // Return false if distance is out of bounds
        bool swapTool();// Return false if distance is out of bounds

        bool homeDrill(); 
        bool gotoDrill(int dist, int speed); // Return false if distance is out of bounds

        prismm_msgs::dam_data getData();

        void eStop();
        void resume();// Continue trilling or moving with motors after e stop
        void reset();// Resume processing but reset motor movements and state

    private:
        ros::NodeHandle nh;

        bool tool_is_drill = true;
        bool drill_is_on = false;
        bool e_stopped = false;
        DamState state = 0;

        HX711 load_cell;
        ACS712 stp_y_current_sensor;
        ACS712_AC drill_current_sensor;

        AccelStepper stp_x1;
        AccelStepper stp_x2;
        AccelStepper stp_y;

}

#endif /** _Dam_H_ **/

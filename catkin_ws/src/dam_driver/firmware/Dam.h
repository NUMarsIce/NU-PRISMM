#ifndef _DAM_H_
#define _DAM_H_

//includes
#include <ACS712.h>
#include <ACS712_AC.h>
#include <AccelStepper.h>
#include <PID_AutoTune_v0.h>
#include <PID_v1.h>

// 
#define RAPID_STP_SPEED 1000
#define DRILL_STP_SPEED 100

#define TOOL_DISTANCE 100

// Stepper 1
#define STP_X_STEP_PIN 6
#define STP_X_DIR_PIN 5
#define STP_X_HOME_PIN 77  

// Stepper 2
#define STP_X_STEP_PIN 8
#define STP_X_DIR_PIN 7
#define STP_X_HOME_PIN 78  

// Load Cells
#define LC1_DAT_PIN 30
#define LC1_CLK_PIN 29

#define LC2_DAT_PIN 28
#define LC2_CLK_PIN 27

/**
 * 
 * Class that represents the drill and movement for PRISMM
 * 
 * */

class Dam {
    public:
        Dam();
        ~Dam();

    enum DamState{
        DEFAULT_STATE = 0,
        HOMING = 1,
        DRILLING = 2,
        PROBING = 3,
        E_STOP = 4,  
    };

    private:


}

#endif /** _Dam_H_ **/

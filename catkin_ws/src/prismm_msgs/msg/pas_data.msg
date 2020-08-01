#time since epoch
time stamp
#current state of the PAS as defined by the PasState enum
int8 state
#load of the load cell on the probe y axis
float32 loadA
float32 loadB
#current to the heating probes in miliamps
float32 drill_current
float32 heat_current
float32 power_current
float32 pow24_current
float32 pow5_current
#thermistors in thousanths of millicentegrade
float23 heat_temp
float23 heat2_temp
float23 ambient_temp
#relays
bool heater
bool heater2
bool power24
bool drill
int8 filter_state
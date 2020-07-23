#time since epoch
time stamp
#current state of the PAS as defined by the PasState enum
int8 state
#load of the load cell on the probe y axis
float32 loadA
float32 loadB
#current to the heating probes in miliamps
uint16 heat_current
uint16 heat2_current
uint16 power_current
uint16 pow24_current
uint16 pow5_current
#thermistors in thousanths of millicentegrade
uint16 heat_temp
uint16 heat2_temp
uint16 ambient_temp
#servo positions in degrees or milimeters
uint16 servo_rot
uint16 servo_ext
#relays
bool heater
bool heater2
bool power24
bool drill

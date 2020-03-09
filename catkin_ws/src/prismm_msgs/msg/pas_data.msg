#time since epoch
time stamp
#current state of the PAS as defined by the PasState enum
int8 state
#load of the load cell on the probe y axis
float32 load
#current to the heating probes in miliamps
uint16 heat_current
uint16 heat2_current
#thermistors in thousanths of millicentegrade
uint16 heat_temp
uint16 heat2_temp
#stepper positions in degrees or milimeters
float32 stp_rot
float32 stp_ext
float32 stp_y

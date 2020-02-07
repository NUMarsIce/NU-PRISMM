
# Imagin RPi's with Ubuntu
TODO

# Changing RPi Hostname
Edit `/etc/hostname`
After a restart it can be conected to with `[new_hostname].local`

# ROS System Variables
On slave computers:
`export ROS_MASTER_URI=http://[rpi_master_hostname].local:11311`

On master Pi:
`export $OS_MASTER_URI=http://[rpi_master_hostname].local:11311`
`export ROS_IP=127.0.0.1`

#Imaging RPi's with MATE 
Mate is prefered to Raspbian as ros is pre-built. As of Jan 2020 it is not available on the RPi 4.
Install from [here](https://ubuntu-mate.org/raspberry-pi/) and install ros [like usual](http://wiki.ros.org/melodic/Installation/Ubuntu)

# Imaging RPi's with buster
Installing packages requires building them, which takes a long time. When possible use Ubuntu/Mate
To install ROS melodic on Rasbian buster [this](https://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi) link and replace "kinetic" with "melodic" and "jessie" with "buster" in all commands

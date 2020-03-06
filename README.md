# RU-PRISMM
The codebase for Northeastern's Mars Ice Team

## TODO

- Firmware
  - Create Classes to represent different sub-systems
  - Incorperate more thermistors
  - Creating configs for different components that are loaded at startup (with params)
    - Ex: steps/mm, data send rate, tool displacment
- RPi Software
  - Create basic MSGs and SRVs
  - Decide on the number of Pis needed
- UI:
  - RQT window for image views
  - RQT window for interaction with prismm
    - Create simple radio button interaction with ROS
    - Decide how to split up window
    - Brainstorm all needed functions
    - Prototype window that interacts with ROS
 - Misc
  - Create datalogger (save to file, csv?)
    - Create (MATLAB) code to view data log
  - Decide what is needed to analyze sensor data to evaluate a digital core
  - Test load cells and thermistors and calibrate them
  

### Notes/Ideas:
- PID speed on the drill (needs electronics and more magnets for better rpm measurement)
- Movements only go through if they wont compromise the device. Ex: cant move X axis while probe is in/close to the ground. Speeds would also be set depending on proximity to the ground. There should be some override incase "wiggling" is needed when underground
  - Motor speed modifier in UI and a override for audo speed adjustment (override unlock direct speed settings)
- Would it be advanagous if all commands were services instead of messages?

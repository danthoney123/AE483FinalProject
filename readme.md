## Client
crazyflie-client/testing/flight_tools.py : \
Functions and classes used in main flight code \
\
crazyflie-client/testing/flightMD.py : \
Secondary flight code mainly used for testing. Drone URIs, movement commands, definitions for drone client and other parameters are defined here. Run to start flying drones. \
\
crazyflie-client/testing/flight_show.py \
Main flight code. Runs music and flight commands. Commands are read off of csv (see time_traveler_cmds.csv for example) \
\
crazyflie-client/testing/flight_analysis.ipynb : \
Main analysis notebook. Allows for easy graphing of user defined flight data. \
\
crazyflie-client/testing/ae483tools.py : \
Defines functioned used in flight_analysis.py \

## Crazyflie Library
crazyflie-lib-python/cflib/crazyflie/mem/__init__.py : \
Modified and simplified function write_LED to allow for faster radio upload speed. \
\
crazyflie-lib-python/cflib/crazyflie/mem/led_driver_memory.py : \
Modified to allow for fast LED changes.

## CrazyFlie Firmware
crazyflie-firmware/src/modules/src/controller/controller_ae483.c : \
Modified to load observer and controller implementations from user specific logic and use different controllers and observers depending on parameters set by DroneClient. Safety function loop is included. \
\
crazyflie-firmware/src/modules/interface/controller/user_specific_logic.h \
Interface for defining controllers and observers specific to each drone. This was done primarily for convenience when working with source control since this file could be ignored and left untouched if group members were flashing to a specific drone. Logic defined in here was used for drone on channel 32. \
\
crazyflie-firmware/src/modules/interface/controller/user_specific_logic_dan.h \
Controller and observer for other drone using channel 16. \
\
crazyflie-firmware/src/deck/drivers/src/ledring12.c : \
Modified a cycle wait function on line 1091 to reduce the time between LED ring mode updates.
## Audio Visualization
crazyflie-client/audio_viz/visualize_audio.ipynb : \
Notebook used to generate necessary LED program csv using mp3 file. Can also produce a video simulation of the LED ring. 
## CSV
crazyflie-client/testing/bad_apple_program.csv : \
LED program required for lightshow. \
\
crazyflie-client/testing/time_traveler_cmds.csv : \
List of positions read by client to translate into movement commands.
## Flight Data

# Python Packages (Not included in AE483 Conda Environment)
PyGame : \
Used for playing audio and retrieving timestamps when a song is playing. Can be installed throught pip. ("pip install pygame") \
\
LibRosa : \
Used for performing the fourier transform needed to translate audio to LED lights. Can be installed through pip. (pip install librosa) 
# External Programs
Pulse Audio (optional): \
External program used to play audio on WSL. Some installations of WSL will play audio fine, others will not. Follow installation instructions here: https://x410.dev/cookbook/wsl/enabling-sound-in-wsl-ubuntu-let-it-sing/ . Double check that path in flight_tools.py is accurate to where it is installed on your system. 




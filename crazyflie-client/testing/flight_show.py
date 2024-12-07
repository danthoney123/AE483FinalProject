###################################
# IMPORTS
from flight_tools import *
from cflib.utils.power_switch import PowerSwitch

# Imports for crazyflie (the drone)
import logging
import time
import json
import numpy as np
import cflib.crtp
from multiprocessing import SimpleQueue
import cflib.crazyflie.mem.led_driver_memory as LEDLib 
from threading import Thread
from threading import Lock
import threading

# Imports for qualisys (the motion capture system)
from threading import Thread

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


###################################
# PARAMETERS

# Specify the uri of the drone to which you want to connect (if your radio
# channel is X, the uri should be 'radio://0/X/2M/E7E7E7E7E7')

# Specify the variables you want to log at 100 Hz from the drone
variables = [
    # State estimates (custom observer)
    'ae483log.p_x',
    'ae483log.p_y',
    'ae483log.p_z',
    'ae483log.psi',
    'ae483log.theta',
    'ae483log.phi',
    'ae483log.v_x',
    'ae483log.v_y',
    'ae483log.v_z',
    # State estimates (default observer)
    # Measurements
    'ae483log.w_x',
    'ae483log.w_y',
    'ae483log.w_z',
    # Desired position (custom controller)
    'ae483log.p_x_des',
    'ae483log.p_y_des',
    'ae483log.p_z_des',
    # Desired position (default controller)
    # Motor power commands
    'ae483log.m_1',
    # Mocap
    'ae483log.p_x_mocap',
    'ae483log.p_y_mocap',
    'ae483log.p_z_mocap',
    'ae483log.psi_mocap',
    'ae483log.theta_mocap',
    'ae483log.phi_mocap',
    # Safety variable
    'extravars.set_motors',
    # New measurments,
    'extravars.v_x_int',
    'extravars.v_y_int',
    'extravars.v_z_int',
    'extravars.p_x_int',
    'extravars.p_y_int',
    'extravars.p_z_int',
    'extravars.mocap_age',
    'extravars.violation_index',
    'extravars.violation_value',
    'extravars.violation_lower',
    'extravars.violation_upper',
    'extravars.current_obs',
    'extravars.psi_des_norm',
    'debugvars.max_mocap_age',
    'debugvars.max_p_dis'
    ]

###################################
# CLIENT FOR CRAZYFLIE
bounds_list = ["n_x","n_y","r",
               "psi","theta","phi",
               "w_x","w_y","w_z",
               "p_x","p_y","p_z",
               "v_x","v_y","v_z",
               "p_x_int", "p_y_int", "p_z_int",
               "v_x_int", "v_y_int", "v_z_int",
               "a_x_in_W", "a_y_in_W","a_z_in_W",
               "flow_age", "r_age", "mocap_age"]

###################################
# FLIGHT CODE
uris = ['radio://0/32/2M/11E7E7E7E7',
        'radio://1/16/2M/22E7E7E7E7']
marker_decks = [30, 10]
filenames = ['drone_1_data', 'drone_2_data']

# uris = ['radio://0/32/2M/11E7E7E7E7',]
# marker_decks = [30]
# filenames = ['drone_1_data']

if __name__ == '__main__':
    print(f'Rebooting drones')
    for uri in uris:
        PowerSwitch(uri).stm_power_cycle()
    time.sleep(5)

    # Initialize radio


    # Example, see list above or controller code for names
    BOUNDS = {'phi_lower': -4.0,
              'phi_upper': 4.0,
              'theta_lower': -4.0,
              'theta_upper': 4.0,
              'w_x_lower': -40.0,
              'w_x_upper': 40.0,
              'w_y_lower': -40.0,
              'w_y_upper': 40.0,
              'a_x_in_W_lower': -100,
              'a_x_in_W_upper': 100,
              'a_y_in_W_lower': -100,
              'a_y_in_W_upper': 100,
              'a_z_in_W_lower': -100,
              'a_z_in_W_upper': 100,
              'mocap_age_upper': 750}

    drone_clients, mocap_clients = start_drones(uris=uris, 
                                    marker_decks=marker_decks,
                                    filenames=filenames, 
                                    variables=variables,
                                    disable_failover=False,
                                    set_bounds=False,
                                    bounds=BOUNDS)
    
    run_show(drone_clients=drone_clients, 
             music_file="60bpm.mp3",
             LED_cmds="60bpm_program.csv",
             drone_cmds="test_cmds.csv",
             drone_offsets=[0, 0],
             music_offset=0.5,
             max_runtime=60)
    
    stop_drones(drone_clients, mocap_clients, bounds_list)
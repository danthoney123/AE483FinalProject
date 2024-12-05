###################################
# IMPORTS
from flight_tools import *

# Imports for crazyflie (the drone)
import logging
import time
import json
import numpy as np
import cflib.crtp
from multiprocessing import SimpleQueue
import cflib.crazyflie.mem.led_driver_memory as LEDLib 
from cflib.utils.power_switch import PowerSwitch

# Imports for qualisys (the motion capture system)
from threading import Thread

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

###################################
# PARAMETERS

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
    'stateEstimate.x',
    'stateEstimate.y',
    'stateEstimate.z',
    'stateEstimate.yaw',
    'stateEstimate.pitch',
    'stateEstimate.roll',
    'stateEstimate.vx',
    'stateEstimate.vy',
    'stateEstimate.vz',
    # Measurements
    'ae483log.w_x',
    'ae483log.w_y',
    'ae483log.w_z',
    'ae483log.n_x',
    'ae483log.n_y',
    'ae483log.r',
    'ae483log.a_z',
    # Desired position (custom controller)
    'ae483log.p_x_des',
    'ae483log.p_y_des',
    'ae483log.p_z_des',
    # Desired position (default controller)
    'ctrltarget.x',
    'ctrltarget.y',
    'ctrltarget.z',
    # Motor power commands
    'ae483log.m_1',
    'ae483log.m_2',
    'ae483log.m_3',
    'ae483log.m_4',
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
    'extravars.a_x',
    'extravars.a_y',
    'extravars.v_x_int',
    'extravars.v_y_int',
    'extravars.v_z_int',
    'extravars.p_x_int',
    'extravars.p_y_int',
    'extravars.p_z_int',
    'extravars.a_x_in_W',
    'extravars.a_y_in_W',
    'extravars.a_z_in_W',
    'extravars.flow_age',
    'extravars.r_age',
    'extravars.mocap_age',
    'extravars.a_x_0',
    'extravars.a_y_0',
    'extravars.a_z_0',
    'extravars.violation_index',
    'extravars.violation_value',
    'extravars.violation_lower',
    'extravars.violation_upper',
    'extravars.current_obs',
    'debugvars.psi_des_norm',
    'debugvars.psi_inter',
    'debugvars.tau_z',
    'debugvars.psi_inter_2'
    ]
# Specify the uri of the drone to which you want to connect (if your radio
# channel is X, the uri should be 'radio://0/X/2M/E7E7E7E7E7')
uri = 'radio://0/32/2M/E7E7E7E7E7' # <-- FIXME
# uri = 'radio://0/32/2M/E7E7E7E7E8' # <-- FIXME

# Specify the IP address of the motion capture system
ip_address = '128.174.245.190' # FIXME

# Specify the name of the rigid body that corresponds to your active marker
# deck in the motion capture system. If your marker deck number is X, this name
# should be 'marker_deck_X'.
# marker_deck_name = 'marker_deck_30' # <-- FIXME
marker_deck_name = 'marker_deck_10'

# Specify the marker IDs that correspond to your active marker deck in the
# motion capture system. If your marker deck number is X, these IDs should be
# [X + 1, X + 2, X + 3, X + 4]. They are listed in clockwise order (viewed
# top-down), starting from the front.
# marker_deck_ids = [31, 32, 33, 34] # FIXME
marker_deck_ids = [11, 12, 13, 14]

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

if __name__ == '__main__':
    print(f'Rebooting drone')
    PowerSwitch(uri).stm_power_cycle()
    time.sleep(5)

    # Specify whether or not to use the motion capture system
    use_mocap = True

    # Example, see full list in _fully_connected() at top of file
    # BOUNDS = {'n_x_lower': -81,
    #           'n_x_upper': 82,
    #           'p_z_lower': -0.1}


    # Initialize radio
    cflib.crtp.init_drivers()

    # Example, see list above or controller code for names
    # BOUNDS = {'w_x_lower': -2.0,
    #           'w_y_lower': -2.0,
    #           'w_x_upper': 2.0,
    #           'w_y_upper': 2.0,
    #           'w_z_lower': -4.0,
    #           'w_z_upper': 4.0,
    #           'psi_lower': -2*np.pi,
    #           'psi_upper': 2*np.pi}

    BOUNDS = {'w_z_lower': -100.0,
              'w_z_upper': 100.0,
              'psi_lower': -12,
              'psi_upper': 12,
              'phi_lower': -1,
              'phi_upper': 1}

    # Create and start the client that will connect to the drone
    drone_client = CrazyflieClient(
        uri,
        use_controller=True, ## If disabled uses default controller
        use_observer=True, ### If disabled uses default observer
        use_safety=True, ### Disable at your own risk
        use_mocap=use_mocap, ### Must have mocap deck installed and mocap system live, set above
        use_LED=True, ### Set to true in all cases where the flow sensor is missing or obstructed
        disable_failover=False, ### If true drone will not switch observers on sensor failure
        set_bounds=True, ### Sends custom bounds to update the defaults
        bounds = BOUNDS,
        bounds_list=bounds_list,
        marker_deck_ids=marker_deck_ids if use_mocap else None,
        filename='hardware_data',
        variables=variables
    )

    # Wait until the client is fully connected to the drone
    while not drone_client.is_fully_connected:
        time.sleep(0.1)
    
    # Create and start the client that will connect to the motion capture system
    if use_mocap:
        pose_queue = SimpleQueue()
        Thread(target=send_poses, args=(drone_client, pose_queue)).start()
        mocap_client = QualisysClient(ip_address, marker_deck_name, pose_queue)
    else:
        mocap_client = None

    # Pause to initiate controller processing
    drone_client.stop(0.1)
    print('Waiting for parameters to be recieved before flight:')
    while(drone_client.params_recieved < drone_client.params_sent):
        time.sleep(0.1)
    print('All bounds recieved, proceeding.')

    # Find offset 
    drone_client.initialize_offset(mocap_obj=mocap_client)
    # drone_client.offset = [0, 0, 0, 0]

    ## Flight code here!
    flight_commands = [
        # Demo flight of the move_frame functionS
        # lambda: drone_client.stop(20)
        lambda: drone_client.move_frame([0, 0, 0.2, 0, "W"], [0, 0, 0.2, 0, "W"], t=1.0),
        lambda: drone_client.move_frame([0, 0, 0.2, 0, "W"], [0, 0, 0.7, 0, "W"], t=3.0),
        # lambda: drone_client.move_frame([0, 0, 0.7, 0, "W"], [0, 0, 0.7, 0, "W"], t=10.0),
        # lambda: drone_client.move_frame([0, 0, 0.7, 0, "W"], [0, 0, 0.7, 0, "W"], t=5.0),
        lambda: drone_client.move_frame([0, 0, 0.7, 0, "W"], [0, 0, 0.7, 0, "W"], t=1.0),
        lambda: drone_client.move_frame([0, 0, 0.7, 0, "W"], [-2.0, 0, 0.7, 0, "G"], t=3.0),
        # lambda: drone_client.move_frame([-2.0, 0, 0.7, 0, "G"], [-2.0, 0, 0.7, 90, "G"], t=3.0),
        lambda: drone_client.move_frame([-2.0, 0, 0.7, 0, "G"], [-2.0, 0, 0.7, 360, "G"], t=10.0),
        lambda: drone_client.move_frame([-2.0, 0, 0.7, 360, "G"], [-2.0, 0, 0.7, 360, "G"], t=3.0),
        # lambda: drone_client.move_frame([-2.0, 0, 0.7, 180, "G"], [-2.0, 0, 0.7, 180, "G"], t=10.0),
        # lambda: drone_client.move_frame([0, 0, 0.7, 180, "W"], [-2.5, 0, 0.7, 180, "G"], t=5.0),
        # lambda: drone_client.move_frame([-2.5, 0, 0.7, 180, "G"], [-2.5, 0, 0.7, 180, "G"], t=3.0),
        # lambda: drone_client.move_frame([-2.5, 0, 0.7, 180, "G"], [0, 0, 0.7, 180, "W"], t=5.0),
        # lambda: drone_client.move_frame([0, 0, 0.7, 90, "W"], [0, 0, 0.7, 180, "W"], t=5.0),
        # lambda: drone_client.move_frame([0, 0, 0.7, 180, "W"], [0, 0, 0.7, 180, "W"], t=10.0),
        # lambda: drone_client.move_frame([0, 0, 0.7, 180, "W"], [0, 0, 0.7, 360, "W"], t=10.0),

        # lambda: drone_client.cf.param.set_value('ae483par.flow_age', 15),
        # lambda: print('Failing Flow'),
        # lambda: time.sleep(2),
        # lambda: drone_client.cf.param.set_value('ae483par.mocap_age', 0),
        # lambda: print('Unfailing mocap'),
        # lambda: time.sleep(5),
        # lambda: drone_client.cf.param.set_value('ae483par.flow_age', 0),
        # lambda: print('Unfailing Flow'),
        # lambda: drone_client.move_frame([0, 0, 0.5, 0, "W"], [0, 0, 0.5, 0, "W"], t=3.0),
        # lambda: drone_client.move_frame([0, 0, 0.5, 0, "W"], [-2.5, 0, 0.6, 0, "G"], t=5.0),
        # lambda: drone_client.move_frame([-2.5, 0, 0.6, 0, "G"], [-2.5, 0, 0.6, 0, "G"], t=3.0),
        # lambda: drone_client.move_frame([-2.5, 0, 0.6, 0, "G"], [0.0, 0, 0.5, 0, "G"], t=5.0),
        # lambda: drone_client.move_frame([-2.5, 0, 0.6, 0, "G"], [0.0, 0, 0.5, 135, "G"], t=10.0),
        # lambda: drone_client.move_frame([-2.5, 0, 0.6, 0, "G"], [-2.5, 0, 0.0, 0, "G"], t=3.0)
        lambda: drone_client.move_frame([-2.0, 0, 0.7, 360, "G"], [-2.0, 0, 0.0, 360, "G"], t=3.0),
    ]

    # Run flight commands
    for command in flight_commands:
        if(drone_client.data['extravars.set_motors']['data'][-1] == 1):
            command()
        else:
            break            

    # # Pause after landing
    drone_client.stop(1.0)

    # Disconnect from the drone
    drone_client.disconnect()

    # Disconnect from the motion capture system
    if use_mocap:
        mocap_client.close()

    # Assemble flight data from both clients
    data = {}
    data['drone'] = drone_client.data
    data['mocap'] = mocap_client.data if use_mocap else {}

    # Write flight data to a file
    with open(drone_client.filename+'.json', 'w') as outfile:
        json.dump(data, outfile, sort_keys=False)

    # Report outcome of flight
    print_outcome(drone_client.data, bounds_list)
            


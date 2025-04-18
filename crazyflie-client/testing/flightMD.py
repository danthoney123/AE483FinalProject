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

shared_lock = Lock()

# Specify the uri of the drone to which you want to connect (if your radio
# channel is X, the uri should be 'radio://0/X/2M/E7E7E7E7E7')
uri_1 = 'radio://0/32/2M/11E7E7E7E7'
uri_2 = 'radio://1/16/2M/22E7E7E7E7' # <-- FIXME


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

# Specify the IP address of the motion capture system
ip_address = '128.174.245.190' # FIXME

marker_deck_name_1 = 'marker_deck_30'
marker_deck_ids_1 = marker_deck_ids = [31, 32, 33, 34]

marker_deck_name_2 = 'marker_deck_10'
marker_deck_ids_2 = marker_deck_ids = [11, 12, 13, 14]

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

    print(f'Rebooting drones')
    # PowerSwitch(uri_1).stm_power_cycle()
    PowerSwitch(uri_2).stm_power_cycle()
    time.sleep(5)
    # Specify whether or not to use the motion capture system
    use_mocap = False

    # Example, see full list in _fully_connected() at top of file
    # BOUNDS = {'n_x_lower': -81,
    #           'n_x_upper': 82,
    #           'p_z_lower': -0.1}


    # Initialize radio
    cflib.crtp.init_drivers()

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

    # Create and start the client that will connect to the drone
    drone_client_1 = CrazyflieClient(
        uri_1,
        use_controller=True, ## If disabled uses default controller
        use_observer=True, ### If disabled uses default observer
        use_safety=False, ### Disable at your own risk
        use_mocap=use_mocap, ### Must have mocap deck installed and mocap system live, set above
        use_LED=True, ### Set to true in all cases where the flow sensor is missing or obstructed
        set_bounds=False, ### Sends custom bounds to update the defaults
        bounds = BOUNDS,
        marker_deck_ids=marker_deck_ids_1 if use_mocap else None,
        bounds_list=bounds_list,
        filename='md_LED_test_D1_time_traveler',
        variables=variables,
        disable_failover=False
    )

    drone_client_2 = CrazyflieClient(
        uri_2,
        use_controller=True, ## If disabled uses default controller
        use_observer=True, ### If disabled uses default observer
        use_safety=False, ### Disable at your own risk
        use_mocap=use_mocap, ### Must have mocap deck installed and mocap system live, set above
        use_LED=True, ### Set to true in all cases where the flow sensor is missing or obstructed
        set_bounds=False, ### Sends custom bounds to update the defaults
        bounds = BOUNDS,
        marker_deck_ids=marker_deck_ids_2 if use_mocap else None,
        bounds_list=bounds_list,
        filename='md_LED_test_D2_time_traveler',
        variables=variables,
        disable_failover=False
    )

    drone_clients = [drone_client_1, drone_client_2]

    # Wait until the client is fully connected to the drone
    while (not drone_client_1.is_fully_connected or not drone_client_2.is_fully_connected):
        time.sleep(0.1)
    
    mocap_clients = []
    # Create and start the client that will connect to the motion capture system
    if use_mocap:
        for drone_client in drone_clients:
            pose_queue = SimpleQueue()
            Thread(target=send_poses, args=(drone_client, pose_queue)).start()
            mocap_clients.append(QualisysClient(ip_address, "marker_deck_" + str(drone_client.marker_deck_ids[0] - 1), pose_queue))
    else:
        mocap_clients = []

    # Pause to initiate controller processing
    if (use_mocap):
        for i, drone_client in enumerate(drone_clients):
            drone_client.stop(0.1)
            print('Waiting for parameters to be recieved before flight:')
            while(drone_client.params_recieved < drone_client.params_sent):
                time.sleep(0.1)
            print('All bounds recieved, proceeding.')
            # Find offset 
            drone_client.initialize_offset(mocap_obj=mocap_clients[i])

    ## Flight code here!
    base_height = 0.7
    def getFlightCommands():
        # flight_commands_1 = [
        #     #!!!!!!!!!!!!!! START UP AND GO UP TO HEIGHT !!!!!!!!!!!!!!!!!!!!!!!!!!
        #     lambda dc: dc.stop(3),
        #     lambda dc: dc.move_frame(p_1 = [0, 0, 0.0, 0, "W"], p_2=[0, 0, 0.2, 0, "W"], t=1.0),
        #     lambda dc: dc.move_frame([0, 0, base_height, 0, "W"], t=5.0),
        #     lambda dc: dc.move_frame([0, 0, base_height, 0, "W"], t=1.0),
        #     lambda dc: dc.move_frame([-2, 0.5, base_height, 0, "Q"], t=5.0),
        #     lambda dc: dc.move_frame([-2, 0.5, base_height, 0, "Q"], t=1.0),
        #     # #!!!!!!!!!!!!!!MOVE IN SQUARE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        #     lambda dc: dc.move_frame([-3, 0.5, base_height, 90, "Q"], t=5.0), # Move
        #     lambda dc: dc.move_frame([-3, 0.5, base_height, 90, "Q"], t=1.0), # Stop
        #     lambda dc: dc.move_frame([-3, -0.5, base_height, 180, "Q"], t=5.0), # Move
        #     lambda dc: dc.move_frame([-3, -0.5, base_height, 180, "Q"], t=1.0), # Stop
        #     lambda dc: dc.move_frame([-2, -0.5, base_height, 270, "Q"], t=5.0), # Move
        #     lambda dc: dc.move_frame([-2, -0.5, base_height, 270, "Q"], t=1.0), # Stop
        #     lambda dc: dc.move_frame([-2, 0.5, base_height, 360, "Q"], t=5.0), # Move
        #     lambda dc: dc.move_frame([-2, 0.5, base_height, 360, "Q"], t=5.0), # Stop

        #     #!!!!!!!!!!!!!!!!!LANDING !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        #     lambda dc: dc.move_frame([-2, 0.5, 0, 360, "Q"], t=5.0),
        # ]

        # flight_commands_2 = [
        #     #!!!!!!!!!!!!!! START UP AND GO UP TO HEIGHT !!!!!!!!!!!!!!!!!!!!!!!!!!
        #     lambda dc: dc.stop(3),
        #     lambda dc: dc.move_frame(p_1 = [0, 0, 0.0, 0, "W"], p_2=[0, 0, 0.2, 0, "W"], t=1.0),
        #     lambda dc: dc.move_frame([0, 0, base_height, 0, "W"], t=5.0),
        #     lambda dc: dc.move_frame([0, 0, base_height, 0, "W"], t=1.0),
        #     lambda dc: dc.move_frame([-3, -0.5, base_height, 0, "Q"], t=5.0),
        #     lambda dc: dc.move_frame([-3, -0.5, base_height, 0, "Q"], t=1.0),
        #     # #!!!!!!!!!!!!!!MOVE IN SQUARE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        #     lambda dc: dc.move_frame([-2, -0.5, base_height, 90, "Q"], t=5.0), # Move
        #     lambda dc: dc.move_frame([-2, -0.5, base_height, 90, "Q"], t=1.0), # Stop
        #     lambda dc: dc.move_frame([-2, 0.5, base_height, 180, "Q"], t=5.0), # Move
        #     lambda dc: dc.move_frame([-2, 0.5, base_height, 180, "Q"], t=1.0), # Stop
        #     lambda dc: dc.move_frame([-3, 0.5, base_height, 270, "Q"], t=5.0), # Move
        #     lambda dc: dc.move_frame([-3, 0.5, base_height, 270, "Q"], t=1.0), # Stop
        #     lambda dc: dc.move_frame([-3, -0.5, base_height, 360, "Q"], t=5.0), # Move
        #     lambda dc: dc.move_frame([-3, -0.5, base_height, 360, "Q"], t=5.0), # Stop
 
        #     #!!!!!!!!!!!!!!!!!LANDING !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        #     lambda dc: dc.move_frame([-3, -0.5, 0, 360, "Q"], t=5.0),
        # ]

        # flight_commands_1 = [
        #     lambda dc: dc.stop(3),
        #     lambda dc: dc.move_frame(p_1 = [0, 0, 0.0, 0, "W"], p_2=[0, 0, 0.2, 0, "W"], t=1.0),
        #     lambda dc: dc.move_frame([0, 0, base_height, 0, "W"], t=5.0),
        #     lambda dc: dc.move_frame([0, 0, base_height, 0, "W"], t=10.0),
        #     lambda dc: dc.move_frame([0, 0, base_height, 360, "W"], t=10.0),
        #     lambda dc: dc.move_frame([0, 0, 0.0, 360, "W"], t=5.0),
        # ]

        # flight_commands_2 = [
        #     lambda dc: dc.stop(3),
        #     lambda dc: dc.move_frame(p_1 = [0, 0, 0.0, 0, "W"], p_2=[0, 0, 0.2, 0, "W"], t=1.0),
        #     lambda dc: dc.move_frame([0, 0, base_height, 0, "W"], t=5.0),
        #     lambda dc: dc.move_frame([0, 0, base_height, 0, "W"], t=10.0),
        #     lambda dc: dc.move_frame([0, 0, base_height, 360, "W"], t=10.0),
        #     lambda dc: dc.move_frame([0, 0, 0.0, 360, "W"], t=5.0),
        # ]

        flight_commands_1 = [

            lambda dc: dc.stop(15)
        ]
        
        flight_commands_2 = [

            lambda dc: dc.stop(15)
        ]

        return [flight_commands_1, flight_commands_2]


    
    

    def drone_thread(drone_client, flight_commands, end_call):
        for command in flight_commands:
            if(drone_client.data['extravars.set_motors']['data'][-1] == 1):
                #with lock:
                command(drone_client)
            else:
                end_call.set()
                break  

    allfcs = getFlightCommands()
    flight_commands_1 = allfcs[0]
    flight_commands_2 = allfcs[1]

    song_file = "time_traveler_shortened.mp3"
    cmd_file = "time_traveler_short_program.csv"
    runtime = 15
    offset = 0.5


    threads = []
    t1_done = threading.Event()
    t1 = Thread(target=drone_thread, args=(drone_client_1, flight_commands_1, t1_done))
    t1.start()
    threads.append(t1)

    t2_done = threading.Event()
    t2 = Thread(target=drone_thread, args=(drone_client_2, flight_commands_2, t2_done))
    t2.start()
    threads.append(t2)

    music_thread = Thread(target=play_song_multidrone, args=([drone_client_1, drone_client_2], [t1_done, t2_done], cmd_file, song_file, runtime, offset))
    music_thread.start()
    threads.append(music_thread)

    for t in threads:
        t.join()

    # Disconnect from the drone
    for drone_client in drone_clients:
        drone_client.disconnect()

    # Disconnect from the motion capture system
    if use_mocap:
        for mocap_client in mocap_clients:
            mocap_client.close()

    # Assemble flight data from both clients
    data_1 = {}
    data_2 = {}
    data_1['drone'] = drone_client_1.data
    data_1['mocap'] = mocap_clients[0].data if use_mocap else {}
    data_2['drone'] = drone_client_2.data
    data_2['mocap'] = mocap_clients[1].data if use_mocap else {}

    # Write flight data to a file
    with open(drone_client_1.filename+'.json', 'w') as outfile:
        json.dump(data_1, outfile, sort_keys=False)

    with open(drone_client_2.filename+'.json', 'w') as outfile:
        json.dump(data_2, outfile, sort_keys=False)

    # Report outcome of flight
    for i, drone_client in enumerate(drone_clients):
        print(f"Outcome of drone {i+1}")
        print_outcome(drone_client.data, bounds_list)
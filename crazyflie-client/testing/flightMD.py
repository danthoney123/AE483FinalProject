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
from threading import Thread
from threading import Lock

# Imports for qualisys (the motion capture system)
from threading import Thread

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


###################################
# PARAMETERS

shared_lock = Lock()

# Specify the uri of the drone to which you want to connect (if your radio
# channel is X, the uri should be 'radio://0/X/2M/E7E7E7E7E7')
uri_1 = 'radio://0/32/2M/E7E7E7E7E8' # <-- FIXME
uri_2 = 'radio://0/32/2M/E7E7E7E7E7'

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
    'extravars.violation_upper'
    ]

# Specify the IP address of the motion capture system
ip_address = '128.174.245.190' # FIXME

marker_deck_name_1 = 'marker_deck_10'
marker_deck_ids_1 = marker_deck_ids = [11, 12, 13, 14]

marker_deck_name_2 = 'marker_deck_30'
marker_deck_ids_2 = marker_deck_ids = [31, 32, 33, 34]

###################################
# CLIENT FOR CRAZYFLIE
bounds_list = ["n_x","n_y","r",
               "psi","theta","phi",
               "w_x","w_y","w_z",
               "p_x","p_y","p_z",
               "v_x","v_y","v_z",
               "p_x_int", "p_y_int", "p_z_int",
               "v_x_int", "v_y_int", "v_z_int",
               "a_x_in_W", "a_y_in_W","a_z_in_W"]

###################################
# FLIGHT CODE

if __name__ == '__main__':
    # Specify whether or not to use the motion capture system
    use_mocap = True

    # Example, see full list in _fully_connected() at top of file
    # BOUNDS = {'n_x_lower': -81,
    #           'n_x_upper': 82,
    #           'p_z_lower': -0.1}


    # Initialize radio
    cflib.crtp.init_drivers()

    # Example, see list above or controller code for names
    BOUNDS = {'w_x_lower': -2.0,
              'w_y_lower': -2.0,
              'w_x_upper': 2.0,
              'w_y_upper': 2.0,
              'w_z_lower': -4.0,
              'w_z_upper': 4.0,
              'psi_lower': -2*np.pi,
              'psi_upper': 2*np.pi}

    # Create and start the client that will connect to the drone
    drone_client_1 = CrazyflieClient(
        uri_1,
        use_controller=True, ## If disabled uses default controller
        use_observer=True, ### If disabled uses default observer
        use_safety=False, ### Disable at your own risk
        use_mocap=use_mocap, ### Must have mocap deck installed and mocap system live, set above
        use_LED=True, ### Set to true in all cases where the flow sensor is missing or obstructed
        set_bounds=True, ### Sends custom bounds to update the defaults
        bounds = BOUNDS,
        marker_deck_ids=marker_deck_ids_1 if use_mocap else None,
        filename='md_flight_test_D1',
        variables=variables
    )

    drone_client_2 = CrazyflieClient(
        uri_2,
        use_controller=True, ## If disabled uses default controller
        use_observer=True, ### If disabled uses default observer
        use_safety=True, ### Disable at your own risk
        use_mocap=use_mocap, ### Must have mocap deck installed and mocap system live, set above
        use_LED=True, ### Set to true in all cases where the flow sensor is missing or obstructed
        set_bounds=False, ### Sends custom bounds to update the defaults
        bounds = BOUNDS,
        marker_deck_ids=marker_deck_ids_2 if use_mocap else None,
        filename='md_flight_test_D2',
        variables=variables
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
            Thread(target=send_poses, args=(drone_client, pose_queue, shared_lock)).start()
            mocap_clients.append(QualisysClient(ip_address, "marker_deck_" + str(drone_client.marker_deck_ids[0] - 1), pose_queue))
    else:
        mocap_clients = []

    # Pause to initiate controller processing
    for i, drone_client in enumerate(drone_clients):
        drone_client.stop(0.1)
        if drone_client.set_bounds:
            print('Waiting for bounds to be recieved before flight:')
            while(drone_client.params_sent < len(BOUNDS) and drone_client.set_bounds):
                time.sleep(0.1)
            print('All bounds recieved, proceeding.')
        else:
            time.sleep(2.0)
            print('No bounds to send, proceeding.')

        # Find offset 
        drone_client.initialize_offset(mocap_obj=mocap_clients[i])

    ## Flight code here!
    def getFlightCommands():
        flight_commands_1 = [
            # Demo flight of the move_frame functionS
            lambda dc: dc.move_frame([0, 0, 0.2, 0, "W"], [0, 0, 0.2, 0, "W"], t=1.0, lock=shared_lock),
            lambda dc: dc.move_frame([0, 0, 0.2, 0, "W"], [0, 0, 0.5, 0, "W"], t=1.0, lock=shared_lock),
            # lambda: drone_client.move_frame([0, 0, 0.5, 0, "W"], [0, 0, 0.5, 0, "W"], t=3.0),
            # lambda: drone_client.move_frame([0, 0, 0.5, 0, "W"], [-2.5, 0, 0.6, 0, "G"], t=5.0),
            # lambda: drone_client.move_frame([-2.5, 0, 0.6, 0, "G"], [-2.5, 0, 0.6, 0, "G"], t=3.0),
            # # lambda: drone_client.move_frame([-2.5, 0, 0.6, 0, "G"], [-2.5, 0, 0.6, 180, "G"], t=2.0),
            # # lambda: drone_client.move_frame([-2.5, 0, 0.6, 180, "G"], [-2.5, 0, 0.6, 0, "G"], t=2.0),
            # lambda: drone_client.move_frame([-2.5, 0, 0.6, 0, "G"], [-2.5, 0, 0.6, 0, "G"], t=5.0),
            # lambda: drone_client.move_frame([-2.5, 0, 0.6, 0, "G"], [-2.5, 0, 0.0, 0, "G"], t=3.0)
        ]
        flight_commands_2 = [
            # Demo flight of the move_frame functionS
            lambda dc: dc.move_frame([0, 0, 0.2, 0, "W"], [0, 0, 0.2, 0, "W"], t=1.0, lock=shared_lock),
            lambda dc: dc.move_frame([0, 0, 0.2, 0, "W"], [0, 0, 0.5, 0, "W"], t=1.0, lock=shared_lock),
            # lambda: drone_client.move_frame([0, 0, 0.5, 0, "W"], [0, 0, 0.5, 0, "W"], t=3.0),
            # lambda: drone_client.move_frame([0, 0, 0.5, 0, "W"], [-2.5, 0, 0.6, 0, "G"], t=5.0),
            # lambda: drone_client.move_frame([-2.5, 0, 0.6, 0, "G"], [-2.5, 0, 0.6, 0, "G"], t=3.0),
            # # lambda: drone_client.move_frame([-2.5, 0, 0.6, 0, "G"], [-2.5, 0, 0.6, 180, "G"], t=2.0),
            # # lambda: drone_client.move_frame([-2.5, 0, 0.6, 180, "G"], [-2.5, 0, 0.6, 0, "G"], t=2.0),
            # lambda: drone_client.move_frame([-2.5, 0, 0.6, 0, "G"], [-2.5, 0, 0.6, 0, "G"], t=5.0),
            # lambda: drone_client.move_frame([-2.5, 0, 0.6, 0, "G"], [-2.5, 0, 0.0, 0, "G"], t=3.0)
        ]
        return [flight_commands_1, flight_commands_2]


    def drone_thread(drone_client, flight_commands, lock):
        for command in flight_commands:
            if(drone_client.data['extravars.set_motors']['data'][-1] == 1):
                #with lock:
                command(drone_client)
            else:
                break  

    allfcs = getFlightCommands()
    flight_commands_1 = allfcs[0]
    flight_commands_2 = allfcs[1]

    threads = []
    t1 = Thread(target=drone_thread, args=(drone_client_1, flight_commands_1, shared_lock))
    t1.start()
    threads.append(t1)

    t2 = Thread(target=drone_thread, args=(drone_client_2, flight_commands_2, shared_lock))
    t2.start()
    threads.append(t2)

    for t in threads:
        t.join()
    
    # for t in threads:
    #     t.close()

    # Run flight commands
    # for command in flight_commands:
    #     if(drone_client.data['extravars.set_motors']['data'][-1] == 1):
    #         command()
    #     else:
    #         break            

    # # # Pause after landing
    # drone_client.stop(1.0)

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
    for drone_client in drone_clients:
        print_outcome(drone_client.data, bounds_list)
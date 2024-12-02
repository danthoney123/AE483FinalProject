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

# Imports for qualisys (the motion capture system)
from threading import Thread

# Other imports
import ast  # For safely evaluating tuples in string form
import csv
import atexit

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


###################################
# PARAMETERS

# Specify the uri of the drone to which you want to connect (if your radio
# channel is X, the uri should be 'radio://0/X/2M/E7E7E7E7E7')
uri = 'radio://0/32/2M/E7E7E7E7E7' # <-- FIXME

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
    ]

# Specify the IP address of the motion capture system
ip_address = '128.174.245.190' # FIXME

# Specify the name of the rigid body that corresponds to your active marker
# deck in the motion capture system. If your marker deck number is X, this name
# should be 'marker_deck_X'.
marker_deck_name = 'marker_deck_30' # <-- FIXME

# Specify the marker IDs that correspond to your active marker deck in the
# motion capture system. If your marker deck number is X, these IDs should be
# [X + 1, X + 2, X + 3, X + 4]. They are listed in clockwise order (viewed
# top-down), starting from the front.
marker_deck_ids = [31, 32, 33, 34] # FIXME

###################################
# CLIENT FOR CRAZYFLIE
bounds_list = {
    "n_x_lower": -80.0, "n_x_upper": 80.0,
    "n_y_lower": -80.0, "n_y_upper": 80.0,
    "r_lower": 0.0, "r_upper": 1.0,
    "psi_lower": -1.0, "psi_upper": 1.0,
    "theta_lower": -0.2, "theta_upper": 0.2,
    "phi_lower": -0.2, "phi_upper": 0.2,
    "w_x_lower": -1.0, "w_x_upper": 1.0,
    "w_y_lower": -1.0, "w_y_upper": 1.0,
    "w_z_lower": -1.5, "w_z_upper": 1.5,
    "p_x_lower": -1.0, "p_x_upper": 1.0,
    "p_y_lower": -1.0, "p_y_upper": 1.0,
    "p_z_lower": 0.0, "p_z_upper": 1.5,
    "v_x_lower": -1.0, "v_x_upper": 1.0,
    "v_y_lower": -1.0,"v_y_upper": 1.0,
    "v_z_lower": -0.5, "v_z_upper": 0.75,
    "p_x_int_lower": -1.0, "p_x_int_upper": 1.0,
    "p_y_int_lower": -1.0, "p_y_int_upper": 1.0,
    "p_z_int_lower": -0.1, "p_z_int_upper": 1.5,
    "v_x_int_lower": -1.5, "v_x_int_upper": 1.5,
    "v_y_int_lower": -1.5, "v_y_int_upper": 1.5,
    "v_z_int_lower": -1.0, "v_z_int_upper": 1.0,
    "a_x_in_W_lower": -4.0, "a_x_in_W_upper": 4.0,
    "a_y_in_W_lower": -4.0, "a_y_in_W_upper": 4.0,
    "a_z_in_W_lower": 4.0, "a_z_in_W_upper": 18.0
}

###################################
# FLIGHT CODE

if __name__ == '__main__':
    # Specify whether or not to use the motion capture system
    use_mocap = False

    # Initialize radio
    cflib.crtp.init_drivers()

    # Example, see list above or controller code for names
    BOUNDS = {'w_x_lower': -2.0,
              'w_y_lower': -2.0,
              'w_x_upper': 2.0,
              'w_y_upper': 2.0,
              'w_z_lower': -4.0,
              'w_z_upper': 4.0,
              'psi_lower': -np.pi,
              'psi_upper': np.pi}

    # Create and start the client that will connect to the drone
    drone_client = CrazyflieClient(
        uri,
        use_controller=False, ## If disabled uses default controller
        use_observer=False, ### If disabled uses default observer
        use_safety=False, ### Disable at your own risk
        use_mocap=use_mocap, ### Must have mocap deck installed and mocap system live, set above
        use_LED=True, ### Set to true in all cases where the flow sensor is missing or obstructed
        set_bounds=False, ### Sends custom bounds to update the defaults
        bounds_list=bounds_list,
        bounds = BOUNDS,
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
    if drone_client.set_bounds:
        print('Waiting for bounds to be recieved before flight:')
        while(drone_client.params_sent < len(BOUNDS) and drone_client.set_bounds):
            time.sleep(0.1)
        print('All bounds recieved, proceeding.')
    else:
        time.sleep(2.0)
        print('No bounds to send, proceeding.')

    # Find offset 
    drone_client.initialize_offset(mocap_obj=mocap_client)

    ## Flight code here!
    flight_commands = [
        # lambda: some command
    ]


    # Run flight commands
    for command in flight_commands:
        if(drone_client.data['extravars.set_motors']['data'][-1] == 1):
            command()
        else:
            break

####################### LED Driver Setup ############################    
    memory = drone_client.cf.mem
    drone_client.cf.param.set_value('ring.effect', 13)
    leds_driver = LEDLib.LEDDriverMemory(id=4, type='LED driver', size=24, mem_handler=memory)
################### Import LED Program ###############################
    # Initialize lists for times and LED states
    times = []
    led_data = []
    # Read the CSV file with correct parsing
    with open("bad_apple_program.csv", mode='r') as file:
        reader = csv.reader(file)
        headers = next(reader)  # Skip headers
        for row in reader:
            # Extract time (last column) and convert to float
            time_instance = float(row[-1])
            # Extract LED data (all columns except the last) and convert each string to an RGB tuple
            led_row = [ast.literal_eval(led) for led in row[:-1]]
            # Append to respective lists
            times.append(time_instance)
            led_data.append(led_row)

########################### Run the show ############################
    # Start pulseaudio for windows capture of audio output
    pulseaudio_process = run_pulseaudio()

    # Play the music and send LED lights
    get_music_time, stop_music, is_playing = play_music("bad_apple.mp3")
    last_led_index = -1
    start_time = time.time()
    first_light_time = 0
    while (last_led_index+1 < len(led_data)) and is_playing() and (time.time()-start_time < 4*60):
        #print(get_music_time())
        current_time = get_music_time() - 0.195
        if current_time >= times[last_led_index+1]:
            led_sum = 0
            for i, led in enumerate(leds_driver.leds):
                led.r = led_data[last_led_index+1][i][0]
                led.g = led_data[last_led_index+1][i][1]
                led.b = led_data[last_led_index+1][i][2]
                led_sum += led.r + led.g + led.b
            if led_sum > 0 and first_light_time == 0:
                first_light_time = current_time
            leds_driver.write_data(None)
            last_led_index += 1
        time.sleep(0.005) #0.005

    # Stop music and close pulseaudio   
    stop_music()
    if pulseaudio_process is not None:
        atexit.register(pulseaudio_process.terminate)
        print('Pulseaudio terminated.')

    print(f'The LEDs first turned on at {first_light_time}s in song time')

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
    print('==========================================================================')
    if drone_client.data['extravars.set_motors']['data'][-1] == 0:
        shutoff_index = np.where((np.array(drone_client.data['extravars.set_motors']['data'][:-1]) == 1) & (np.array(drone_client.data['extravars.set_motors']['data'][1:]) == 0))[0][0]
        shutoff_time = drone_client.data['extravars.set_motors']['time'][shutoff_index]
        shutoff_keys = ['p_z_des', 'm_1', 'm_2', 'm_3', 'm_4', 'p_x_des', 'p_y_des']
        shutoff_data = []
        for key in shutoff_keys:
            closest_index =  np.abs(np.array(drone_client.data['ae483log.'+key]['time']) - shutoff_time).argmin()
            shutoff_data.append(drone_client.data['ae483log.'+key]['data'][closest_index])
        if np.sum(shutoff_data) != 0:
            print(f'Drone shutoff occured during flight at {shutoff_time} seconds when p_z_des was {shutoff_data[0]} \nand average motor command was {np.average(shutoff_data[1:5])}.')
            print(f'P_x_des was {shutoff_data[5]} and p_y_des was {shutoff_data[6]}.')
        else:
            print(f'Drone shutoff occured at {shutoff_time} seconds but was on the ground or landing.')
        if drone_client.data['extravars.violation_index']['data'][-1] != -1:
            keys_avaliable = list(bounds_list.keys())
            bound_violated = (keys_avaliable[2*drone_client.data['extravars.violation_index']['data'][-1]]).removesuffix('_lower')
            if bound_violated+'_lower' in BOUNDS and drone_client.set_bounds:
                bound_lower = BOUNDS[bound_violated+'_lower']
            else:
                bound_lower = bounds_list[bound_violated+'_lower']
            if bound_violated+'_upper' in BOUNDS and drone_client.set_bounds:
                bound_upper = BOUNDS[bound_violated+'_upper']
            else:
                bound_upper = bounds_list[bound_violated+'_upper']
            print('\n')
            print(f"The {bound_violated} bound was violated with value {drone_client.data['extravars.violation_value']['data'][-1]}")
            print(f"The acceptable range of {bound_violated} is [{bound_lower}, {bound_upper}]")
        else:
            print('\n')
            print(f"The drone did not record any bounds violation (last data was recorded at {drone_client.data['extravars.violation_index']['time'][-1]}).\nTry extending the last stop command.")
    else:
        print('The drone had a "successful" flight.')
    print('==========================================================================')
            


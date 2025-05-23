###################################
# IMPORTS

# Imports for crazyflie (the drone)
import logging
import time
import json
import numpy as np
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from multiprocessing import SimpleQueue
import struct
import threading

# Imports for qualisys (the motion capture system)
import asyncio
import xml.etree.cElementTree as ET
from threading import Thread
import qtm_rt as qtm
from scipy.spatial.transform import Rotation

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


###################################
# PARAMETERS

# Specify the uri of the drone to which you want to connect (if your radio
# channel is X, the uri should be 'radio://0/X/2M/E7E7E7E7E7')
uri = 'radio://0/16/2M/E7E7E7E7E7' # <-- FIXME

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
marker_deck_name = 'marker_deck_10' # <-- FIXME

# Specify the marker IDs that correspond to your active marker deck in the
# motion capture system. If your marker deck number is X, these IDs should be
# [X + 1, X + 2, X + 3, X + 4]. They are listed in clockwise order (viewed
# top-down), starting from the front.
marker_deck_ids = [11, 12, 13, 14] # FIXME

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

class CrazyflieClient:
    def __init__(self, uri, use_controller=False, use_observer=False, use_mocap=False, use_safety=True, marker_deck_ids=None, set_bounds = False, bounds={}):
        self.use_controller = use_controller
        self.use_observer = use_observer
        self.use_mocap = use_mocap
        self.set_bounds = set_bounds
        self.bounds = bounds
        self.marker_deck_ids = marker_deck_ids
        self.cf = Crazyflie(rw_cache='./cache')
        self.cf.connected.add_callback(self._connected)
        self.cf.fully_connected.add_callback(self._fully_connected)
        self.cf.connection_failed.add_callback(self._connection_failed)
        self.cf.connection_lost.add_callback(self._connection_lost)
        self.cf.disconnected.add_callback(self._disconnected)
        self.use_safety = use_safety
        print(f'CrazyflieClient: Connecting to {uri}')
        self.cf.open_link(uri)
        self.is_fully_connected = False
        self.data = {}
        self.params_sent = 0
        self.offset = [0, 0, 0, 0]

    def float_to_custom(self, int_value, float_value):
        # Get the IEEE 754 binary representation of the float
        [f_bits] = struct.unpack('>I', struct.pack('>f', float_value))
        
        # Extract sign (1 bit), exponent (8 bits), and mantissa (23 bits)
        sign = (f_bits >> 31) & 0x1
        exponent = (f_bits >> 23) & 0xFF
        mantissa = f_bits & 0x7FFFFF

        # Adjust exponent to fit in 6 bits and mantissa in 19 bits
        custom_exponent = max(0, min((exponent - 127 + 31), 0x3F))  # Bias to 31, fit in 6 bits
        custom_mantissa = mantissa >> 4  # Truncate mantissa to 19 bits

        # Reassemble the custom float
        custom_float_bits = (sign << 25) | (custom_exponent << 19) | custom_mantissa
        
        int_value &= 0x3F  # 6 bits, max 63
        combined = (int_value << 26) | custom_float_bits
        return combined

    def _connected(self, uri):
        print(f'CrazyflieClient: Connected to {uri}')
    
    def _fully_connected(self, uri):
        if self.marker_deck_ids is not None:
            print(f'CrazyflieClient: Using active marker deck with IDs {marker_deck_ids}')

            # Set the marker mode (3: qualisys)
            self.cf.param.set_value('activeMarker.mode', 3)

            # Set the marker IDs
            self.cf.param.set_value('activeMarker.front', marker_deck_ids[0])
            self.cf.param.set_value('activeMarker.right', marker_deck_ids[1])
            self.cf.param.set_value('activeMarker.back', marker_deck_ids[2])
            self.cf.param.set_value('activeMarker.left', marker_deck_ids[3])

        # Reset the default observer
        self.cf.param.set_value('kalman.resetEstimation', 1)
        time.sleep(0.1)
        self.cf.param.set_value('kalman.resetEstimation', 0)
        
        # Reset the ae483 observer, safety bounds, and backup state estimation
        self.cf.param.set_value('ae483par.reset', 1)
        print('Reset Called')

        # Set use mocap
        if self.use_mocap:
            self.cf.param.set_value('ae483par.use_mocap', 1)
        else:
            self.cf.param.set_value('ae483par.use_mocap', 0)
        print('Setting mocap')

        if self.use_safety:
            self.cf.param.set_value('ae483par.use_safety', 1)
            print('Turning safety on.')
        else:
            self.cf.param.set_value('ae483par.use_safety', 0)
            print('Turned safety off, watch out everyone!')

        # Reset the ae483 safe_to_set_motors value. Thank you for using safety features!
        self.cf.param.set_value('ae483par.set_motors', 1)
        time.sleep(0.1)
        
        # Enable the controller (1 for default, 6 for ae483)
        if self.use_controller:
            self.cf.param.set_value('stabilizer.controller', 6)
        else:
            self.cf.param.set_value('stabilizer.controller', 1)

        #thing to fix not working without flow deck ***************************
        self.cf.param.set_value('stabilizer.estimator', 2)

        # Enable the observer (0 for disable, 1 for enable)
        if self.use_observer:
            self.cf.param.set_value('ae483par.use_observer', 1)
        else:
            self.cf.param.set_value('ae483par.use_observer', 0)

        def param_callback(name, value):
            print(f'Parameter {name} recieved with value {value}.')
            self.params_sent += 1

        if self.set_bounds:
            for key in self.bounds.keys():
                keys_avaliable = list(bounds_list.keys())
                print(f'Setting new bound for {key} at index {keys_avaliable.index(key)}')
                self.cf.param.add_update_callback(group='ae483par', name='bounds_update', cb=param_callback)
                self.cf.param.set_value('ae483par.bounds_update', self.float_to_custom(keys_avaliable.index(key), self.bounds[key]))
                time.sleep(0.1) # Give time for controller to update value

        # Start logging
        self.logconfs = []
        self.logconfs.append(LogConfig(name=f'LogConf0', period_in_ms=10))
        num_variables = 0
        for v in variables:
            num_variables += 1
            if num_variables > 5: # <-- could increase if you paid attention to types / sizes (max 30 bytes per packet)
                num_variables = 0
                self.logconfs.append(LogConfig(name=f'LogConf{len(self.logconfs)}', period_in_ms=10))
            self.data[v] = {'time': [], 'data': []}
            self.logconfs[-1].add_variable(v)
        for logconf in self.logconfs:
            try:
                self.cf.log.add_config(logconf)
                logconf.data_received_cb.add_callback(self._log_data)
                logconf.error_cb.add_callback(self._log_error)
                logconf.start()
            except KeyError as e:
                print(f'CrazyflieClient: Could not start {logconf.name} because {e}')
                for v in logconf.variables:
                    print(f' - {v.name}')
            except AttributeError:
                print(f'CrazyflieClient: Could not start {logconf.name} because of bad configuration')
                for v in logconf.variables:
                    print(f' - {v.name}')
        
        print(f'CrazyflieClient: Fully connected to {uri}')
        self.is_fully_connected = True

    def _connection_failed(self, uri, msg):
        print(f'CrazyflieClient: Connection to {uri} failed: {msg}')

    def _connection_lost(self, uri, msg):
        print(f'CrazyflieClient: Connection to {uri} lost: {msg}')

    def _disconnected(self, uri):
        print(f'CrazyflieClient: Disconnected from {uri}')
        self.is_fully_connected = False
    
    def _log_data(self, timestamp, data, logconf):
        for v in logconf.variables:
            self.data[v.name]['time'].append(timestamp / 1e3)
            self.data[v.name]['data'].append(data[v.name])

    def _log_error(self, logconf, msg):
        print(f'CrazyflieClient: Error when logging {logconf}: {msg}')

    def move(self, x, y, z, yaw, dt):
        print(f'CrazyflieClient: Move to {x}, {y}, {z} with yaw {yaw} degrees for {dt} seconds')
        start_time = time.time()
        while time.time() - start_time < dt:
            self.cf.commander.send_position_setpoint(x+self.offset[0], y+self.offset[1], z+self.offset[2], yaw+self.offset[3])
            time.sleep(0.1)
    
    def stop(self, dt):
        print(f'CrazyflieClient: Stop for {dt} seconds')
        self.cf.commander.send_stop_setpoint()
        self.cf.commander.send_notify_setpoint_stop()
        start_time = time.time()
        while time.time() - start_time < dt:
            time.sleep(0.1)

    def disconnect(self):
        self.cf.close_link()

    def move_smooth(self, p_inW_1, p_inW_2, yaw, v):
        print(f'Move smoothly from {p_inW_1} to {p_inW_2} with yaw {yaw} degrees at {v} meters / second')

        # Make sure p_inW_1 and p_inW_2 are numpy arrays
        p_inW_1 = np.array(p_inW_1)
        p_inW_2 = np.array(p_inW_2)
        
        # Compute distance from p_inW_1 to p_inW_2
        d = np.linalg.norm(p_inW_2-p_inW_1)              # <-- FIXME (A)
        
        # Compute time it takes to move from p_inW_1 to p_inW_2 at desired speed
        dt = d/v             # <-- FIXME (B)
        
        # Get start time
        start_time = time.time()

        # Repeat until the current time is dt seconds later than the start time
        while True:
            # Get the current time
            t = time.time()
            
            # Compute what fraction of the distance from p_inW_1 to p_inW_2
            # should have been travelled by the current time
            s = (t - start_time)/dt          # <-- FIXME (C)
            
            # Compute where the drone should be at the current time, in the
            # coordinates of the world frame
            p_inW_des = (1-s)*p_inW_1 + s*p_inW_2  # <-- FIXME (D)
            
            # Send the desired position (and yaw angle) to the drone
            self.cf.commander.send_position_setpoint(p_inW_des[0]+self.offset[0], p_inW_des[1]+self.offset[1], p_inW_des[2]+self.offset[2], yaw+self.offset[3])

            # Stop if the move is complete (i.e., if the desired position is at p_inW_2)
            # otherwise pause for 0.1 seconds before sending another desired position
            if s >= 1:
                return
            else:
                time.sleep(0.1)

    def initialize_offset(self):
        print('Stopping drone for 3 seconds to start data collection, initialize drone, and collect mocap data')
        self.stop(3.0)
        if self.use_mocap:
            x_arr = np.array(self.data['ae483log.p_x_mocap']['data'])
            y_arr = np.array(self.data['ae483log.p_y_mocap']['data'])
            z_arr = np.array(self.data['ae483log.p_z_mocap']['data'])
            psi_arr = np.array(self.data['ae483log.psi_mocap']['data'])
            mocap_x = x_arr[~np.isnan(x_arr)][-1] if np.any(~np.isnan(x_arr)) else None
            mocap_y = y_arr[~np.isnan(y_arr)][-1] if np.any(~np.isnan(y_arr)) else None
            mocap_z = z_arr[~np.isnan(z_arr)][-1] if np.any(~np.isnan(z_arr)) else None
            mocap_psi = psi_arr[~np.isnan(psi_arr)][-1] if np.any(~np.isnan(psi_arr)) else None
            self.offset = [mocap_x, mocap_y, mocap_z, mocap_psi] # Zero psi for now, drone must be start facing x+
            if any(x is None for x in self.offset):
                raise Exception('No mocap data!')
            self.offset[2] -= (2.29967 + 0.28033)/100.0
            self.offset[3] = np.rad2deg(self.offset[3])
            print(f'Found an offset of {self.offset}')
        else:
            print('Skipping intialization, mocap not used so defaulting to drone frame.')


###################################
# CLIENT FOR QUALISYS

class QualisysClient(Thread):
    def __init__(self, ip_address, marker_deck_name, pose_queue):
        Thread.__init__(self)
        self.ip_address = ip_address
        self.marker_deck_name = marker_deck_name
        self.connection = None
        self.qtm_6DoF_labels = []
        self._stay_open = True
        self.data = {
            'time': [],
            'x': [],
            'y': [],
            'z': [],
            'yaw': [],
            'pitch': [],
            'roll': [],
        }
        self.pose_queue = pose_queue
        self.start()

    def close(self):
        self.pose_queue.put('END')
        self._stay_open = False
        self.join()

    def run(self):
        asyncio.run(self._life_cycle())

    async def _life_cycle(self):
        await self._connect()
        while (self._stay_open):
            await asyncio.sleep(1)
        await self._close()

    async def _connect(self):
        print('QualisysClient: Connect to motion capture system')
        self.connection = await qtm.connect(self.ip_address, version='1.24')
        params = await self.connection.get_parameters(parameters=['6d'])
        xml = ET.fromstring(params)
        self.qtm_6DoF_labels = [label.text.strip() for index, label in enumerate(xml.findall('*/Body/Name'))]
        await self.connection.stream_frames(
            components=['6d'],
            on_packet=self._on_packet,
        )

    def _on_packet(self, packet):
        header, bodies = packet.get_6d()
        
        if bodies is None:
            print(f'QualisysClient: No rigid bodies found')
            return
        
        if self.marker_deck_name not in self.qtm_6DoF_labels:
            print(f'QualisysClient: Marker deck {self.marker_deck_name} not found')
            return
         
        index = self.qtm_6DoF_labels.index(self.marker_deck_name)
        position, orientation = bodies[index]

        # Get time in seconds, with respect to the qualisys clock
        t = packet.timestamp / 1e6

        # Get position of marker deck (x, y, z in meters)
        x, y, z = np.array(position) / 1e3
        
        # Get orientation of marker deck (yaw, pitch, roll in radians)
        R = Rotation.from_matrix(np.reshape(orientation.matrix, (3, -1), order='F'))
        yaw, pitch, roll = R.as_euler('ZYX', degrees=False)

        # Store time, position, and orientation
        self.data['time'].append(t)
        self.data['x'].append(x)
        self.data['y'].append(y)
        self.data['z'].append(z)
        self.data['yaw'].append(yaw)
        self.data['pitch'].append(pitch)
        self.data['roll'].append(roll)

        # Check if the measurements are valid
        if np.isfinite(x):
            # Convert orientation to quaternion
            qx, qy, qz, qw = R.as_quat()
            # Check if the queue of measurements is empty. We do this because
            # we do not want to create a backlog of measurements. I bet there
            # is a better way of handling this - to *replace* the measurement
            # in the queue with the current one. Revisit this another time!
            if self.pose_queue.empty():
                # Put pose in queue to send to the drone
                self.pose_queue.put((x, y, z, qx, qy, qz, qw))        

    async def _close(self):
        await self.connection.stream_frames_stop()
        self.connection.disconnect()

def send_poses(client, queue):
    print('Start sending poses')
    while True:
        pose = queue.get()
        if pose == 'END':
            print('Stop sending poses')
            break
        x, y, z, qx, qy, qz, qw = pose
        client.cf.extpos.send_extpose(x, y, z, qx, qy, qz, qw)

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
              'psi_lower': -np.pi,
              'psi_upper': np.pi}

    # Create and start the client that will connect to the drone
    drone_client = CrazyflieClient(
        uri,
        use_controller=True,
        use_observer=False,
        use_safety=False,
        use_mocap=use_mocap,
        set_bounds=False,
        bounds = BOUNDS,
        marker_deck_ids=marker_deck_ids if use_mocap else None,
    )

    # Wait until the client is fully connected to the drone
    while not drone_client.is_fully_connected:
        time.sleep(0.1)
    
    # Create and start the client that will connect to the motion capture system
    if use_mocap:
        pose_queue = SimpleQueue()
        Thread(target=send_poses, args=(drone_client, pose_queue)).start()
        mocap_client = QualisysClient(ip_address, marker_deck_name, pose_queue)

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
    ### Put flight code below here! ###

    # Just stop, please, just stop
    # drone_client.stop(3.0)

    # Find offset 
    drone_client.initialize_offset()

    ## Flight code here!
    flight_commands = [
        # short hop
        # lambda: drone_client.move(0.0, 0.0, 0.2, 0.0, 1.0),
        # lambda: drone_client.move_smooth([0.0, 0.0, 0.2], [0.0, 0.0, 0.5], 0.0, 0.2),
        lambda: drone_client.stop(5)

        # # # yaw test
        # # lambda: drone_client.move(0.0, 0.0, 0.5, -35.0, 3.0),
        # # lambda: drone_client.move(0.0, 0.0, 0.5, 0.0, 3.0),

        # # # hover
        # lambda: drone_client.move(0.0, 0.0, 0.5, 0.0, 3.0),

        # # # move in square
        # lambda: drone_client.move_smooth([0.0, 0.0, 0.5], [0.5, 0.0, 0.5], 0.0, 0.2),
        # lambda: drone_client.move(0.5, 0.0, 0.5, 0.0, 3.0),
        # lambda: drone_client.move_smooth([0.5, 0.0, 0.5], [0.5, 0.5, 0.5], 0.0, 0.2),
        # lambda: drone_client.move(0.5, 0.5, 0.5, 0.0, 3.0),
        # lambda: drone_client.move_smooth([0.5, 0.5, 0.5], [0.0, 0.5, 0.5], 0.0, 0.2),
        # lambda: drone_client.move(0.0, 0.5, 0.5, 0.0, 3.0),
        # lambda: drone_client.move_smooth([0.0, 0.5, 0.5], [0.0, 0.0, 0.5], 0.0, 0.2),
        # lambda: drone_client.move(0.0, 0.0, 0.5, 0.0, 3.0),

        # # # land
        # lambda: drone_client.move_smooth([0.0, 0.0, 0.5], [0.0, 0.0, 0.0], 0.0, 0.2),
        
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
    with open('../crazyflie-client/FinalProject/flight_test_01.json', 'w') as outfile:
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
            


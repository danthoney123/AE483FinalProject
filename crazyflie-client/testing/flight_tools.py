# Imports for crazyflie (the drone)
import logging
import time
import json
import numpy as np
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
import struct
import os

# Imports for qualisys (the motion capture system)
import asyncio
import xml.etree.cElementTree as ET
from threading import Thread
import qtm_rt as qtm
from scipy.spatial.transform import Rotation

# Other imports
import pygame
import subprocess

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

class CrazyflieClient:
    def __init__(self, uri, use_controller=False, use_observer=False, use_mocap=False, use_LED = True, use_safety=True, marker_deck_ids=None, set_bounds = False, bounds_list=None, bounds={}, filename='hardware_data', variables=None):
        self.use_controller = use_controller
        self.use_observer = use_observer
        self.use_mocap = use_mocap
        self.use_safety = use_safety
        self.use_LED = use_LED
        self.set_bounds = set_bounds
        self.bounds = bounds
        self.marker_deck_ids = marker_deck_ids
        self.cf = Crazyflie(rw_cache='./cache')
        self.cf.connected.add_callback(self._connected)
        self.cf.fully_connected.add_callback(self._fully_connected)
        self.cf.connection_failed.add_callback(self._connection_failed)
        self.cf.connection_lost.add_callback(self._connection_lost)
        self.cf.disconnected.add_callback(self._disconnected)
        print(f'CrazyflieClient: Connecting to {uri}')
        self.cf.open_link(uri)
        self.is_fully_connected = False
        self.data = {}
        self.params_sent = 0
        self.offset = [0, 0, 0, 0]
        self.filename=filename
        self.bounds_list = bounds_list
        self.variables = variables

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
            print(f'CrazyflieClient: Using active marker deck with IDs {self.marker_deck_ids}')

            # Set the marker mode (3: qualisys)
            self.cf.param.set_value('activeMarker.mode', 3)

            # Set the marker IDs
            self.cf.param.set_value('activeMarker.front', self.marker_deck_ids[0])
            self.cf.param.set_value('activeMarker.right', self.marker_deck_ids[1])
            self.cf.param.set_value('activeMarker.back', self.marker_deck_ids[2])
            self.cf.param.set_value('activeMarker.left', self.marker_deck_ids[3])

        # Set which estimator is being used! Very important AAAAA
        print('Using LED Deck Setting Estimator')
        if self.use_LED:
            self.cf.param.set_value('stabilizer.estimator', 2)
        else:
            self.cf.param.set_value('stabilizer.estimator', 1)

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

        if self.use_LED:
            self.cf.param.set_value('ae483par.use_LED', 1)
        else:
            self.cf.param.set_value('ae483par.use_LED', 0)
        print('Setting LED Deck')

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
                keys_avaliable = list(self.bounds_list.keys())
                print(f'Setting new bound for {key} at index {keys_avaliable.index(key)}')
                self.cf.param.add_update_callback(group='ae483par', name='bounds_update', cb=param_callback)
                self.cf.param.set_value('ae483par.bounds_update', self.float_to_custom(keys_avaliable.index(key), self.bounds[key]))
                time.sleep(0.1) # Give time for controller to update value

        # Start logging
        self.logconfs = []
        self.logconfs.append(LogConfig(name=f'LogConf0', period_in_ms=10))
        num_variables = 0
        for v in self.variables:
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

    def move_frame(self, p_1, p_2, t):
        # New smooth move command with format pX = [x, y, z, psi [degrees!!!!!!!!], "Frame"]
        print(f'Move smoothly from {p_1} to {p_2} in {t} seconds.')
        pos_1 = np.array(p_1[:3])
        pos_2 = np.array(p_2[:3])
        frame_1 = p_1[4]
        frame_2 = p_2[4]
        psi_1 = np.array(p_1[3])
        psi_2 = np.array(p_2[3])

        if frame_1 == "W":
            pos_1_Q = pos_1 + np.array(self.offset[:3])
            psi_1_Q = psi_1 + self.offset[3]
        else:
            pos_1_Q = pos_1
            psi_1_Q = psi_1
        if frame_2 == "W":
            pos_2_Q = pos_2 + np.array(self.offset[:3])
            psi_2_Q = psi_2 + self.offset[3]
        else:
            pos_2_Q = pos_2
            psi_2_Q = psi_2

        if t == 0:
            t = 1

        dpos = pos_2_Q - pos_1_Q
        dpsi = psi_2_Q - psi_1_Q
        v = dpos/t
        w = dpsi/t

        start_time = time.time()
        while time.time() - start_time <= t:
            current_time = time.time() - start_time
            pos_des = current_time*v + pos_1_Q
            psi_des = current_time*w + psi_1_Q
            self.cf.commander.send_position_setpoint(pos_des[0], pos_des[1], pos_des[2], psi_des)
            time.sleep(0.1)
      
    def initialize_offset(self, mocap_obj=None):
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
            if any(x is None for x in self.offset) or all(x == 0 for x in self.offset):
                print(f'Error: no mocap data! Offset was {self.offset}. Saving and exiting.')
                self.disconnect()
                mocap_obj.close()
                data = {}
                data['drone'] = self.data
                data['mocap'] = mocap_obj.data
                with open(self.filename+'.json', 'w') as outfile:
                    json.dump(data, outfile, sort_keys=False)
                os._exit(1)
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
        # print(f'Sending {pose}')


def play_music(file_path):
    pygame.mixer.init()
    pygame.mixer.music.load(file_path)
    time.sleep(1)
    pygame.mixer.music.play()

    def get_music_time():
        return pygame.mixer.music.get_pos()/1000
    def stop_music():
        return pygame.mixer.music.stop()
    def is_playing():
        return pygame.mixer.music.get_busy()    
    
    return get_music_time, stop_music, is_playing

def run_pulseaudio(path_to_pulse=r'/mnt/c/pulseaudio-1.1/bin/pulseaudio.exe'):
    result = subprocess.run(
        ["pgrep", "-f", path_to_pulse],
        capture_output=True, text=True
    )
    if result.stdout.strip():
        print('Pulseaudio is already running.')
        return None
    else:
        process = subprocess.Popen(
            ["nohup", path_to_pulse, '--use-pid-file=false', '-D'],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setpgrp
        )
        time.sleep(3.0)
        print("Pulseaudio started")
        return process
    
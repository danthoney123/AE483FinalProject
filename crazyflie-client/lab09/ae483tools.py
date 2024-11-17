import json
import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt

def load_hardware_data(filename):
    with open(Path(filename), 'r') as f:
        data = json.load(f)
        return data['drone'], data['mocap']
    

def resample_data_drone(raw_data, hz=1e2, t_min_offset=0., t_max_offset=0.):
    # copy data (FIXME: may be unnecessary?)
    data = {}
    for key, val in raw_data.items():
        data[key] = {
            'time': val['time'].copy(),
            'data': val['data'].copy(),
        }

    # convert lists to numpy arrays
    for key, val in data.items():
        val['data'] = np.array(val['data'], dtype=np.float64).copy()
        val['time'] = np.array(val['time'], dtype=np.float64).copy()

    # find time interval
    t_min = -np.inf
    t_max = np.inf
    for key, val in data.items():
        t_min = max(t_min, val['time'][0])
        t_max = min(t_max, val['time'][-1])
    if (t_min_offset < 0) or (t_min_offset > t_max - t_min):
        raise Exception(f't_min_offset = {t_min_offset:.4f} must be in [0, {t_max - t_min:.4f}]')
    if (t_max_offset < 0) or (t_max_offset > t_max - t_min):
        raise Exception(f't_max_offset = {t_max_offset:.4f} must be in [0, {t_max - t_min:.4f}]')
    t_min += t_min_offset
    t_max -= t_max_offset

    # find zero time
    t_zero = t_min

    # do time shift
    t_min -= t_zero
    t_max -= t_zero
    for key, val in data.items():
        val['time'] -= t_zero

    # create an array of times at which to subsample
    if np.round(hz) != hz:
        raise Exception(f'hz = {hz} must be an integer')
    nt = int(1 + np.floor((t_max - t_min) * hz))
    t = t_min + np.arange(0, nt) / hz
    
    # resample raw data with linear interpolation
    resampled_data = {'time': t}
    for key, val in data.items():
        resampled_data[key] = np.interp(t, val['time'], val['data'])
        
    # return the resampled data
    return resampled_data


def resample_data_mocap(raw_data, t, t_shift=0.):
    # copy data (FIXME: may be unnecessary?) and convert lists to numpy arrays
    data = {}
    for key, val in raw_data.items():
        data[key] = np.array(val, dtype=np.float64).copy()
    
    # find nan
    is_valid = np.ones_like(data['time']).astype('bool')
    for val in data.values():
        is_valid = np.bitwise_and(is_valid, ~np.isnan(val))
    i_is_valid = np.argwhere(is_valid).flatten()

    # remove nan
    for key, val in data.items():
        data[key] = val[i_is_valid]
    
    # do time shift
    data['time'] -= (data['time'][0] - t_shift)

    # resample raw data with linear interpolation
    resampled_data = {'time': t.copy()}
    for key, val in data.items():
        if key != 'time':
            resampled_data[key] = np.interp(t, data['time'], val)
    
    # return the resampled data
    return resampled_data

def transform_data_mocap(raw_data):
    # Copy raw data
    data = {}
    for key, val in raw_data.items():
        data[key] = val.copy()

    # Define parameters
    d_1 = 0.0229967 # <-- FIXME
    d_2 = 0.0028033 # <-- FIXME
    
    # Pose of drone body frame in active marker frame
    R_BinA = np.eye(3)                   # <-- FIXME
    p_BinA = np.array([0., 0., -d_1])      # <-- FIXME

    ####################################
    # START OF ANALYSIS AT TIME STEP 0
    #
    
    # Pose of drone world frame in active marker frame (valid t=0 only)
    R_WinA = np.eye(3)                   # <-- FIXME
    p_WinA = np.array([0., 0., -d_1-d_2])      # <-- FIXME

    # Get measurements of (x, y, z) and (psi, theta, phi) from mocap
    x, y, z = data['x'][0], data['y'][0], data['z'][0]
    psi, theta, phi = data['yaw'][0], data['pitch'][0], data['roll'][0]

    # Pose of active marker frame in mocap world frame (valid any t)
    R_AinQ = np.array([[np.cos(psi), -np.sin(psi), 0],
                        [np.sin(psi), np.cos(psi),  0],
                        [0,           0,            1]])@np.array([
                        [np.cos(theta),  0, np.sin(theta)],
                        [0,              1,             0],
                        [-np.sin(theta), 0, np.cos(theta)]])@np.array([
                        [1, 0,                      0],
                        [0, np.cos(phi), -np.sin(phi)],
                        [0, np.sin(phi), np.cos(phi)]])                   # <-- FIXME
    p_AinQ = np.array([x, y, z])      # <-- FIXME
    
    # Pose of drone world frame in mocap world frame (valid t=0 only)
    R_WinQ = R_AinQ @ R_WinA                   # <-- FIXME
    p_WinQ = p_AinQ + R_AinQ @ p_WinA      # <-- FIXME
    
    # Pose of mocap world frame in drone world frame (valid t=0 only)
    R_QinW = (R_WinQ).T                   # <-- FIXME
    p_QinW = -(R_WinQ).T @ p_WinQ      # <-- FIXME

    #
    # END OF ANALYSIS AT TIME STEP 0
    ####################################

    for i in range(len(data['time'])):

        ####################################
        # START OF ANALYSIS AT TIME STEP i
        #

        # Get measurements of (x, y, z) and (psi, theta, phi) from mocap
        x, y, z = data['x'][i], data['y'][i], data['z'][i]
        psi, theta, phi = data['yaw'][i], data['pitch'][i], data['roll'][i]

        # Pose of active marker deck in mocap world frame
        R_AinQ = np.array([[np.cos(psi), -np.sin(psi), 0],
                            [np.sin(psi), np.cos(psi),  0],
                            [0,           0,            1]])@np.array([
                            [np.cos(theta),  0, np.sin(theta)],
                            [0,              1,             0],
                            [-np.sin(theta), 0, np.cos(theta)]])@np.array([
                            [1, 0,                      0],
                            [0, np.cos(phi), -np.sin(phi)],
                            [0, np.sin(phi), np.cos(phi)]])                   # <-- FIXME
        p_AinQ = np.array([x, y, z])      # <-- FIXME

        # Pos of drone in mocap
        p_BinQ = p_AinQ + (R_BinA.T) @ p_BinA
        R_BinQ = R_AinQ @ R_BinA

        # Pose of drone body frame in drone world frame
        R_BinW = R_QinW @ R_BinQ               # <-- FIXME
        p_BinW = p_QinW + R_QinW @ p_BinQ  # <-- FIXME

        # Replace measurements of (x, y, z) and (phi, theta, psi) from mocap
        data['x'][i], data['y'][i], data['z'][i] = p_BinW[0], p_BinW[1], p_BinW[2]              # <-- FIXME
        r = Rotation.from_matrix(R_BinW)
        data['yaw'][i], data['pitch'][i], data['roll'][i] = r.as_euler('ZYX', degrees=False)     # <-- FIXME

        #
        # END OF ANALYSIS AT TIME STEP i
        ####################################
    
    # Return the result
    return data

def sync_data_mocap(raw_data_mocap, t, z_drone, search_direction='None', manual_t_shift = None):

    def get_RMSE(raw_data_mocap, t, t_shift, z_drone):
        resampled_data_mocap = resample_data_mocap(raw_data_mocap, t, t_shift=t_shift)
        transformed_data_mocap = transform_data_mocap(resampled_data_mocap)
        z_mocap = transformed_data_mocap['z']
        return np.sqrt(np.mean((z_mocap - z_drone)**2))
    
    dt = 0.01 # timestep to increment by
    # init data
    RMSE_zero = get_RMSE(raw_data_mocap, t, 0.0, z_drone)
    RMSE_pos = get_RMSE(raw_data_mocap, t, dt, z_drone)
    RMSE_neg = get_RMSE(raw_data_mocap, t, -dt, z_drone)
    if (RMSE_zero < RMSE_pos) and (RMSE_zero < RMSE_neg):
        t_shift_min=0.
        loop = False
    elif RMSE_neg > RMSE_pos:
        # search forward in time
        RMSE_min = RMSE_pos
        t_shift_min = dt
        loop = True
        inc = dt
    else:
        # search backward in time
        RMSE_min = RMSE_neg
        t_shift_min = -dt
        loop = True
        inc = -dt

    if search_direction =='Positive':
        inc = dt
        RMSE_min = RMSE_pos
    elif search_direction == 'Negative':
        inc = -dt
        RMSE_min = RMSE_neg

    while loop:
        RMSE_new = get_RMSE(raw_data_mocap, t, t_shift_min+inc, z_drone)
        if RMSE_new < RMSE_min:
            RMSE_min = RMSE_new
            t_shift_min += inc
        else:
            break
    
    if not manual_t_shift == None:
        t_shift_min = manual_t_shift

    # Resample mocap data with the time shift that minimizes RMSE
    print(f'Shifting data by {t_shift_min} seconds')
    print(f'Increment for shifting: {inc}')
    print(RMSE_zero, RMSE_pos, RMSE_neg)
    resampled_data_mocap = resample_data_mocap(raw_data_mocap, t, t_shift=t_shift_min)

    # Transform mocap data
    transformed_data_mocap = transform_data_mocap(resampled_data_mocap)

    # Return the result
    return transformed_data_mocap

def only_in_flight(data_drone, data_mocap=None, t_interval=None):
    # Verify the desired z position was logged
    if ('ae483log.p_z_des' not in data_drone.keys()) and ('ctrltarget.z' not in data_drone.keys()):
        raise Exception('Neither "ae483log.p_z_des" or "ctrltarget.z" were logged.')
    
    # Find the indices at which the desired z position was positive
    i = []
    for k in ['ae483log.p_z_des', 'ctrltarget.z']:
        if k in data_drone.keys():
            j = np.argwhere(data_drone[k] > 0).flatten()
            if len(j) > len(i):
                i = j
    
    # Verify that there were indices at which the desired z position was positive
    if len(i) < 2:
        raise Exception('The desired z position was never positive.')
    
    # Get first and last index at which the desired z position was positive
    i_first = i[0]
    i_last = i[-1]

    # Adjust the first and last index, if desired, to get a subset of data with
    # a given length centered in the middle of the flight time (if desired)
    if t_interval is not None:
        # Get time step (assuming it is constant)
        dt = data_drone['time'][1] - data_drone['time'][0]

        # Get number of time steps that would correspond to desired time interval
        n_interval = int(np.ceil(t_interval / dt))
        n_flight = (i_last + 1) - i_first
        
        # Verify that we have at least that number of time steps
        if n_flight < n_interval:
            t_flight = n_flight * dt
            raise Exception(f'The requested time interval ({t_interval:.2f} s) is longer than the flight time ({t_flight:.2f} s).')
        
        # Get first and last index again
        i_first += int(np.floor((n_flight - n_interval) / 2))
        i_last = i_first + n_interval

    # Truncate and time-shift data_drone
    for k in data_drone.keys():
        data_drone[k] = data_drone[k][i_first:(i_last + 1)]
    data_drone['time'] -= data_drone['time'][0]
    
    # Truncate and time-shift data_mocap, if it exists
    if data_mocap is not None:
        # Verify mocap data actually exist
        if len(data_mocap.keys()) == 0:
            raise Exception('The dictionary "data_mocap" is empty.')

        # Truncate mocap data
        for k in data_mocap.keys():
            data_mocap[k] = data_mocap[k][i_first:(i_last + 1)]

        # Time-shift mocap data
        data_mocap['time'] -= data_mocap['time'][0]

        # Verify drone and mocap data are the same
        assert(np.allclose(data_drone['time'], data_mocap['time']))

def check_mocap_coverage(filename):
    # Load data
    raw_data_drone, raw_data_mocap = load_hardware_data(filename)

    # Compute statistics
    how_many_timesteps = len(raw_data_mocap['z'])
    how_many_dropouts = len(np.argwhere(np.isnan(raw_data_mocap['z'])).flatten())
    percent_coverage = 100. * (1. - (how_many_dropouts / how_many_timesteps))
    elapsed_time = raw_data_mocap['time'][-1] - raw_data_mocap['time'][0]
    sample_rate = how_many_timesteps / elapsed_time

    # Plot data (with relevant statistics in the title)
    fig, ax = plt.subplots(1, 1, figsize=(7, 3), tight_layout=True)
    ax.plot(raw_data_mocap['time'], raw_data_mocap['z'])
    ax.set_xlabel('t (seconds)')
    ax.set_ylabel('z (meters)')
    ax.set_ylim(0., 1.)
    ax.set_title(f'({filename}) Tracked {percent_coverage:.1f}% of {how_many_timesteps} time steps at about {sample_rate:.0f} Hz')
    ax.grid()
"""
Manual trajectories for testing several tasks.

python -m deform_realsim.hardware.utils.manual_trajs

@priyasun, @contactrka,

make_fig8 form @junwuz

"""

import numpy as np
from matplotlib import pyplot as plt
import os
from scipy.interpolate import interp1d
from scipy.ndimage import gaussian_filter1d


def smooth_traj(traj, sigma=1.0):
    smoothed = np.copy(traj)
    for d in range(traj.shape[1]):
        smoothed[:, d] = gaussian_filter1d(traj[:, d], sigma=sigma)
    return smoothed


def plot_trajectory(tgt_traj, real_traj=None):
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    pad = 0.05  # pad for plot bounds
    ax.set_xlim3d(tgt_traj[:, 0].min()-pad,
                  tgt_traj[:, 0].max()+pad)
    ax.set_ylim3d(tgt_traj[:, 1].min()-pad,
                  tgt_traj[:, 1].max()+pad)
    ax.set_zlim3d(tgt_traj[:, 2].min()-pad,
                  tgt_traj[:, 2].max()+pad)
    x, y, z = tgt_traj.T
    ax.scatter(x, y, z, 'b')
    if real_traj is not None:
        x, y, z = real_traj.T
        ax.scatter(x, y, z, c='r')
    plt.show()


def make_horizontal_fig8_waypoints(center, n_steps, focal_distance=0.2, n_loops=1):
    all_waypoints = np.zeros((n_loops * n_steps, 2))
    for l in range(n_loops):
        for i in range(n_steps):
            theta = 2 * np.pi * i/n_steps
            # ref: https://en.wikipedia.org/wiki/Lemniscate_of_Bernoulli
            scale = 1 / (1 + np.sin(theta)**2)
            y = scale * focal_distance * np.cos(theta) + center[0]
            x = scale * focal_distance * np.sin(theta) * np.cos(theta) + center[1]
            all_waypoints[l*n_steps + i] = np.array((x, y))
    return all_waypoints.reshape(n_loops*n_steps, 2)


def make_fig8_waypoints(n_steps, center=(0.57623798, 0.00134189, 0.43377915),
                        focal_distance=0.2, n_loops=1):
    all_waypoints = np.zeros((n_loops * n_steps, 3))
    for l in range(n_loops):
        for i in range(n_steps):
            theta = 2 * np.pi * i/n_steps
            # ref: https://en.wikipedia.org/wiki/Lemniscate_of_Bernoulli
            scale = 1 / (1 + np.sin(theta)**2)
            y = scale * focal_distance * np.cos(theta) + center[1]
            z = scale * focal_distance * np.sin(theta) * np.cos(theta) + center[2]
            all_waypoints[l*n_steps + i] = np.array((0.6, y, z))
    return all_waypoints.reshape(n_loops*n_steps, 3)


def make_circular_waypoints(n_steps, center=(0.7, 0.025, 0.1), radius=0.15,
                            n_loops=1):
    all_waypoints = np.zeros((n_loops * n_steps, 3))
    for l in range(n_loops):
        for i in range(n_steps):
            theta = - 2 * np.pi * i/n_steps
            x = radius * np.cos(theta + np.pi) + center[0]
            y = radius * np.sin(theta + np.pi) + center[1]
            all_waypoints[l*n_steps + i] = np.array((x, y, center[2]))
    return all_waypoints

def make_wiping_waypoints(loaded_waypts):
    waypts = smooth_traj(loaded_waypts)
    return waypts

def make_horizontal_wiping_waypoints(center, traj_range, n_steps, n_loops=1):
    '''
    Generates a section of a sine wave for wiping motion
    '''
    all_waypoints = np.zeros((n_loops * n_steps, 2))
    left_end = center[0] - traj_range / 2
    right_end = center[0] + traj_range / 2
    increment = 1/n_steps
    theta = 0

    for l in range(n_loops):
        for i in range(n_steps):
            theta += increment
            if theta >= right_end:
                increment *= -1
            if theta <= left_end:
                increment *= -1
            x = theta
            y = 0.17 * np.cos(15 * theta)
            all_waypoints[l*n_steps + i] = np.array((x, y))
    return all_waypoints.reshape(n_loops*n_steps, 2)


def linear_interp(a, b, i):
    x = (b[0] - a[0])*i + a[0]
    y = (b[1] - a[1])*i + a[1]
    z = (b[2] - a[2])*i + a[2]
    return np.array([x, y, z])


def make_shake_waypoints(start, end, n_loops, n_steps):
    all_waypoints = np.zeros((n_loops * n_steps, 3))
    for l in range(n_loops):
        for i in range(n_steps):
            if i < n_steps/4:  # out fast
                all_waypoints[l * n_steps + i] = linear_interp(
                    start, end, 4*i/n_steps)
            else:  # in slower
                all_waypoints[l * n_steps + i] = linear_interp(
                    end, start, (i/n_steps -.25)*4/3)
    return all_waypoints.reshape(n_loops*n_steps, 3)


def make_traj(waypts, duration_sec=5.0, freq=10, mode='quadratic', plot=False):
    tgt_len = int(duration_sec*freq)
    ids = np.arange(waypts.shape[0])
    interp_i = np.linspace(0, ids.max(), tgt_len)
    xi = interp1d(ids, waypts[:, 0], kind=mode)(interp_i)
    yi = interp1d(ids, waypts[:, 1], kind=mode)(interp_i)
    zi = interp1d(ids, waypts[:, 2], kind=mode)(interp_i)
    pos = np.array([xi, yi, zi]).T  # n_steps x 3D
    dt = duration_sec/tgt_len
    vel = (pos[1:, :] - pos[:-1, :])/dt
    vel = np.vstack([vel, vel[-2:-1]])
    traj = pos
    assert(traj.shape[0] == vel.shape[0])
    return traj, vel


#def make_stretch_waypoints(center, traj_time, control_freq):
#    waypts = np.array([ \
#            center, \
#            center + np.array([-0.15, -0.07, 0.0]), \
#            center + np.array([-0.23, -0.11, -0.05]), \
#            center + np.array([-0.21, -0.15, -0.15]), \
#            ])
#    traj, _ = make_traj(waypts, duration_sec=traj_time, freq=control_freq)
#    print(traj)
#    return traj

def make_stretch_waypoints(approach_point, traj_time, control_freq):
    center = approach_point + np.array([-0.08,0,-0.155])
    waypts = np.array([ \
            approach_point, \
            center, \
            center + np.array([-0.15, -0.07, 0.0]), \
            center + np.array([-0.23, -0.11, -0.05]), \
            center + np.array([-0.21, -0.15, -0.15]), \
            ])
    traj, _ = make_traj(waypts, duration_sec=traj_time, freq=control_freq)
    print(traj)
    return traj

def make_calib_waypoints(center, width, traj_time, control_freq):
    waypts = np.array([ \
            center + np.array([width/2, width/2, 0]), \
            center + np.array([width/2, -width/2, 0]), \
            center + np.array([-width/2, -width/2, 0]), \
            center + np.array([-width/2, width/2, 0]), \
            center + np.array([width/2, width/2, 0]), \
            ])
    traj, _ = make_traj(waypts, duration_sec=traj_time, freq=control_freq, mode='linear')
    print(traj)
    return traj

def make_calib_sweep_traj(center, width, n, traj_time, control_freq, platform_height=0.055):
    waypts = []
    increment = width/n
    y_sweep = np.linspace(center[1] + width/2, center[1] - width/2, n)
    x_sweep = np.linspace(center[0] - width/2, center[0] + width/2, n)
    for y in y_sweep:
        for x in x_sweep:
            waypts.append([x,y,center[2]])
    waypts = np.array(waypts)
    traj, _ = make_traj(waypts, duration_sec=traj_time, freq=control_freq, mode='linear')
    waypts[:,2] = platform_height
    print(traj)
    return traj, waypts

def make_fling_waypoints(traj_time, control_freq):
    #center = np.array([0.7, 0, 0.2])
    center = np.array([0.7, 0, 0.175])
    #waypts = np.array([ \
    #        center, \
    #        center + np.array([0.2, 0, 0.2]), \
    #        center + np.array([0.1, 0, 0.05]), \
    #        center + np.array([0.1, 0, 0]), \
    #        center + np.array([0.05, 0, -0.05]), \
    #        center + np.array([-0.05, 0, -0.05]), \
    #        center + np.array([-0.10, 0, -0.05]),
    #        ])
    waypts = np.array([ \
            center, \
            center + np.array([0.2, 0, 0.2]), \
            center + np.array([0.1, 0, 0.05]), \
            center + np.array([0.0, 0, 0]), \
            center + np.array([-0.08, 0, -0.05]), \
            center + np.array([-0.135, 0, -0.02]),
            ])
    traj, _ = make_traj(waypts, duration_sec=traj_time, freq=control_freq)
    print(traj)
    return traj

def make_arcsim_fling_waypoints(traj_time, control_freq):
    path_to_waypoints = os.path.join(os.path.dirname(__file__), 'trajs', 'lift_traj.npy')
    #path_to_waypoints = os.path.join(os.path.dirname(__file__), 'trajs', 'fold_traj.npy')
    waypts = np.load(path_to_waypoints)
    traj, _ = make_traj(waypts, duration_sec=traj_time, freq=control_freq, mode='cubic')
    print(traj)
    return traj

def make_arcsim_stretch_waypoints(traj_time, control_freq):
    #path_to_waypoints = os.path.join(os.path.dirname(__file__), 'trajs', 'stretch_real_traj.npy')
    path_to_waypoints = os.path.join(os.path.dirname(__file__), 'trajs', 'new_stretch.npy')
    waypts = np.load(path_to_waypoints)
    traj, _ = make_traj(waypts, duration_sec=traj_time, freq=control_freq, mode='cubic')
    print(traj)
    return traj

def translate_waypoints(waypoints, axis, offset):
    waypoints = waypoints.copy()
    if axis == 'x':
        waypoints[:,0] += offset
    elif axis == 'y':
        waypoints[:,1] += offset
    else:
        waypoints[:,2] += offset
    return waypoints


def main():
    plot_trajectory(make_circular_waypoints(n_steps=50))


if __name__ == "__main__":
    main()

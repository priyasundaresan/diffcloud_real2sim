"""
A simple script for making Kinova Gen3 robot follow a given trajectory.

python -m kinova_percep.hardware.scripts.kinova_cart_vel lift ~/code/tmp
python -m kinova_percep.hardware.scripts.kinova_cart_vel fold ~/code/tmp

@priyasundaresan

"""
from datetime import datetime
import argparse
import sys
import time
import os

import numpy as np
np.set_printoptions(suppress=True)

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

from ..utils.manual_trajs import (
    plot_trajectory, make_fig8_waypoints, make_circular_waypoints,
    make_arcsim_lift_waypoints, make_arcsim_fold_waypoints, make_arcsim_stretch_waypoints)
from ..utils.kinova_utils import (
    DeviceConnection, command_velocity, check_hard_boundary, get_position,
    go_home, go_pose)
from ..utils.kinova_with_camera import KinovaWithCamera
from ..utils.kinova_multicamera import KinovaMulticamera
from ..utils.task_info import TASK_INFO


TRAJ_TIME = 5  # seconds
CONTROL_FREQUENCY = 10
CONTROL_DURATION = 1.0/float(CONTROL_FREQUENCY)


def run_trajectory(base, tgt_traj):
    """Execute tgt_traj (N x 3) with a fixed gripper orientation."""
    real_traj = []
    # Run the loop to command robot to follow trajectory
    for step in range(len(tgt_traj)):
        curr_pos,  _ = get_position(base)
        if not check_hard_boundary(base, curr_pos):  # stops the robot
            break
        real_traj.append(curr_pos)
        tgt_vel = (tgt_traj[step] - curr_pos) / CONTROL_DURATION
        curr_step_strt = time.time()
        command_velocity(base, tgt_vel, CONTROL_DURATION)
        curr_step_fnsh = time.time()
        print('CONTROL_DURATION', CONTROL_DURATION, 'sec')
        print('Actual duration', curr_step_strt - curr_step_fnsh, 'sec')
    # Return positions that the real robot reached when executing tgt_traj.
    real_traj = np.stack(real_traj)
    return real_traj


def main():
    if len(sys.argv) < 3:
        print('Usage: kinova_cart_vel task_name /path/to/out/logdir')
        exit(1)
    task_name = sys.argv[1]
    assert(task_name in TASK_INFO.keys())
    tstamp = datetime.strftime(datetime.today(), '%y%m%d_%H%M%S')
    logdir = os.path.join(os.path.expanduser(sys.argv[2]),
                          f'{task_name:s}_{tstamp:s}')

    n_steps = TRAJ_TIME * CONTROL_FREQUENCY
    tgt_traj = None
    tgt_ori_deg = None
    go_home_first = True
    open_gripper_end = False
    force_thresh = None

    if task_name == 'fig8':  # execute figure 8 demo
        tgt_ori_deg = np.array([90, 0, 90])
        tgt_traj = make_fig8_waypoints(n_steps)
    elif task_name == 'lift':
        tgt_ori_deg = np.array([150, 0, 90])
        traj_time = 2.5
        n_steps = traj_time * CONTROL_FREQUENCY
        tgt_traj = make_arcsim_lift_waypoints(traj_time, CONTROL_FREQUENCY)
    elif task_name == 'fold':
        tgt_ori_deg = np.array([150, 0, 90])
        traj_time = 2.5
        n_steps = traj_time * CONTROL_FREQUENCY
        tgt_traj = make_arcsim_fold_waypoints(traj_time, CONTROL_FREQUENCY)
    #elif task_name == 'fling':
    #    go_home_first = False
    #    tgt_ori_deg = np.array([150, 0, 90])
    #    traj_time = 2.5
    #    n_steps = traj_time * CONTROL_FREQUENCY
    #    tgt_traj = make_arcsim_fling_waypoints(traj_time, CONTROL_FREQUENCY)
    elif task_name == 'stretch':
        tgt_ori_deg = np.array([120, -30, 90])
        traj_time = 3
        force_thresh = 50.0
        tgt_traj = make_arcsim_stretch_waypoints(traj_time, CONTROL_FREQUENCY)
    else:
        print('Unknown task', task_name)
        exit(1)

    print('Task', task_name, 'tgt_traj', tgt_traj.shape, 'logdir', logdir)

    task_info = TASK_INFO.get(task_name, None)
    if task_info is None:
        #kincam = KinovaWithCamera(logdir, cam_rec_interval=1)
        kincam = KinovaMulticamera(logdir, cam_rec_interval=1)
    else:
        #kincam = KinovaWithCamera(
        #    logdir, cam_rec_interval=1,
        #    edge_size_px=task_info['edge_size_px'],
        #    max_unmasked_z=task_info['max_unmasked_z'])
        kincam = KinovaMulticamera(
            logdir, cam_rec_interval=1,
            edge_size_px=task_info['edge_size_px'],
            max_unmasked_z=task_info['max_unmasked_z'])

    with DeviceConnection.createTcpConnection() as router:
        with DeviceConnection.createUdpConnection() as router_real_time:
            base = BaseClient(router)
            base_cyclic = BaseCyclicClient(router_real_time)
            if go_home_first:
                print('Moving to home pose')
                assert(go_home(base))
            plot_trajectory(tgt_traj)
            input('Press ENTER to execute trajectory')
            print('Moving to start of tgt_traj')
            assert(go_pose(base, tgt_traj[0], tgt_ori_deg))
            time.sleep(1)
            print('Executing trajectory')
            real_traj = kincam.run_trajectory_record_video(base, base_cyclic, tgt_traj, force_thresh=force_thresh)
            base.Stop()  # stop robot motion
            base.Stop()
            print('Plotting results')
            plot_trajectory(tgt_traj, real_traj)
            return True

if __name__ == "__main__":
    exit(main())

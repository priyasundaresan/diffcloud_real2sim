"""
A simple script for making Kinova Gen3 robot follow a given trajectory.

python -m deform_realsim.hardware.scripts.kinova_cart_vel

@contactrika

"""
import sys
import time
import os

import numpy as np
np.set_printoptions(suppress=True)

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

from ..utils.manual_trajs import (
    plot_trajectory, make_calib_waypoints, make_calib_sweep_traj)
from ..utils.kinova_utils import (
    DeviceConnection, command_velocity, check_hard_boundary, get_position,
    go_home, go_pose)
from ..utils.kinova_with_camera import KinovaWithCamera
from ..utils.task_info import TASK_INFO


#TRAJ_TIME = 8  # seconds
TRAJ_TIME = 35  # seconds
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
    tgt_ori_deg = np.array([150, 0, 90])
    center = np.array([0.65, 0.0, 0.115]) #top down calib
    #center = np.array([0.65, 0.23, 0.115]) # side
    chessboard_width = 0.22 # in meters
    tgt_traj, waypoints = make_calib_sweep_traj(center, chessboard_width, 5, TRAJ_TIME, CONTROL_FREQUENCY)
    np.save('robot_chessboard.npy', waypoints)
    logdir = os.path.expanduser('~/code/tmp/calib')

    kincam = KinovaWithCamera(logdir, cam_rec_interval=1)

    with DeviceConnection.createTcpConnection() as router:
        with DeviceConnection.createUdpConnection() as router_real_time:
            base = BaseClient(router)
            base_cyclic = BaseCyclicClient(router_real_time)
            print('Moving to home pose')
            assert(go_home(base))
            plot_trajectory(tgt_traj)
            input('Press ENTER to go to start pose')
            print('Moving to start of tgt_traj')
            assert(go_pose(base, tgt_traj[0], tgt_ori_deg))
            time.sleep(1)
            input('Press ENTER to execute trajectory')
            print('Executing trajectory')
            # real_traj = run_trajectory(base, tgt_traj)
            #real_traj = kincam.run_trajectory_record_video(base, tgt_traj)
            real_traj = kincam.run_trajectory_record_video(base, base_cyclic, tgt_traj)
            base.Stop()  # stop robot motion
            print('Plotting results')
            plot_trajectory(tgt_traj, real_traj)
            return True


if __name__ == "__main__":
    exit(main())

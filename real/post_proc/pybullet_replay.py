"""
Kinova robot loading in sim.

python3 -m deform_realsim.demos.sim_kinova_demo

@contactrika

"""
import argparse
import os

import numpy as np
import pybullet
import pybullet_data
import pybullet_utils.bullet_client as bclient

from utils.bullet_manipulator import BulletManipulator
from utils.camera_utils import MultiCamera
from utils.pybullet_recorder import PyBulletRecorder

def get_args(parent=None):
    parser = argparse.ArgumentParser(description='Main', add_help=False)
    parser.add_argument(
        '--joint_angles_file', type=str,
        default='/Users/priyasundaresan/fling_soft17_211009_202945/traj.npz',
        help='Path to file with N x 7 joint angle trajectory.')
    args = parser.parse_args()
    return args


def main(args):
    #
    # Init Pybullet sim.
    #
    sim = bclient.BulletClient(connection_mode=pybullet.GUI)
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, False)
    pybullet.configureDebugVisualizer(
        pybullet.COV_ENABLE_RGB_BUFFER_PREVIEW, True)
    pybullet.configureDebugVisualizer(
        pybullet.COV_ENABLE_DEPTH_BUFFER_PREVIEW, True)
    pybullet.configureDebugVisualizer(
        pybullet.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, True)
    cam_args = {
        'cameraDistance': 2.0, 'cameraYaw': 0, 'cameraPitch': -30,
        'cameraTargetPosition': np.array([0, 0, 0])
    }
    sim.resetDebugVisualizerCamera(**cam_args)
    # sim.resetSimulation(pybullet.RESET_USE_DEFORMABLE_WORLD)
    sim.setGravity(0, 0, -9.8)
    sim.setTimeStep(1.0/500.0)
    #
    # Load trajectory from file.
    #
    #traj_data = np.load(os.path.expanduser(args.joint_angles_file))
    traj_data = np.load(os.path.expanduser(args.joint_angles_file))['qpos_deg_traj']
    print('Loaded traj_data', traj_data.shape)
    assert(traj_data.shape[-1] == 7)  # 7DoF joint poses
    traj_data = traj_data*np.pi/180  # degrees to radians
    #
    # Load the robot URDF.
    #
    curr_dir = os.path.dirname(os.path.realpath(__file__))
    robot_path = os.path.join(curr_dir, 'data', 'kinova_robot',
                              'gen3_robotiq_2f_85.urdf')
    print('Loading robot from', robot_path)
    robot = BulletManipulator(
        sim, robot_path, control_mode='velocity',
        ee_joint_name='gripper_base_joint',
        ee_link_name='end_effector_link',
        base_pos=[0,0,0],
        base_quat=pybullet.getQuaternionFromEuler([0, 0, 0]),
        global_scaling=1.0,
        use_fixed_base=True,
        debug=True)
    # Set fingers to be permanently closed.
    fing_ids = []
    for i in range(sim.getNumJoints(robot.info.robot_id)):
        jid, jname, jtype, *_ = sim.getJointInfo(robot.info.robot_id, i)
        jname = jname.decode("utf-8")
        if (('finger' in jname or 'knuckle' in jname) and
                jtype == pybullet.JOINT_REVOLUTE):
            fing_ids.append(jid)
    print('fing_ids', fing_ids)
    # left_outer_knuckle Blue
    fing_jpos_lows = [0,    0,      -0.8757, 0,    0,      -0.8757]
    fing_jpos_highs = [0.8, 0.8757,  0.8757, 0.81, 0.8757,  0.8757]
    fing_jpos = [0.7,  # left_outer_finger_joint Cyan
                 -0.6,    # left_inner_finger_joint Red
                 0.82,  # left_inner_knuckle_joint Yellow
                 ]*2  # mimic joint angles
    #
    # Load floor plane and rigid objects.
    #
    sim.setAdditionalSearchPath(pybullet_data.getDataPath())
    floor_id = sim.loadURDF('plane.urdf')
    # obj_path = os.path.join(parent_dir, 'data', 'misc', 'pole.urdf')
    # print('Loading object', obj_path)
    # obj_id = pybullet.loadURDF(
    #     fileName=obj_path,
    #     basePosition=(0.4, 0, 1.0), baseOrientation=(0,0,0,1))
    # Enable rendering after all the objects are loaded.
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 1)
    view_matrix, proj_matrix = MultiCamera.get_cam_vals(
        cam_rolls=[0], cam_yaws=[cam_args['cameraYaw']],
        cam_pitches=[cam_args['cameraPitch']],
        cam_dist=cam_args['cameraDistance'],
        cam_target=cam_args['cameraTargetPosition'],
        proj_matrix=None,  # get_cam_vals will compute proj_matrix
        fov=90, aspect_ratio=1.0)[0][:2]
    #
    # Make recorder to export pkl files.
    #
    recorder = PyBulletRecorder()
    #
    #
    # Run simulation.
    #
    recorder.register_object(robot.info.robot_id, robot_path)
    # recorder.register_object(obj_id, obj_path)
    for step in range(traj_data.shape[0]):
        qpos = traj_data[step]
        print('step', step, 'qpos', qpos)
        robot.reset_to_qpos(qpos)
        for j, jpos in zip(fing_ids, fing_jpos):
            sim.resetJointState(bodyUniqueId=robot.info.robot_id,
                                jointIndex=j, targetValue=jpos,
                                targetVelocity=0)
        sim.stepSimulation()
        MultiCamera.render(
            sim, cam_rolls=[0], cam_yaws=[cam_args['cameraYaw']],
            cam_pitches=[cam_args['cameraPitch']],
            cam_dist=cam_args['cameraDistance'],
            cam_target=cam_args['cameraTargetPosition'],
            proj_matrix=proj_matrix, fov=90, width=300,
            return_seg=True, debug=True)
        recorder.add_keyframe()
    fnm = os.path.expanduser(args.joint_angles_file[:-4]+'_recorder.pkl')
    recorder.save(fnm)
    print('PyBullet Recorder saved', fnm)


if __name__ == '__main__':
    main(get_args())

"""
Utility functions for simple Cartesian velocity control for Kinova robot.

@contactrika
"""
import time
import pprint
import argparse
import threading

import numpy as np

from kortex_api.TCPTransport import TCPTransport
from kortex_api.UDPTransport import UDPTransport
from kortex_api.RouterClient import RouterClient, RouterClientSendOptions
from kortex_api.SessionManager import SessionManager
from kortex_api.autogen.messages import Session_pb2, Base_pb2, BaseCyclic_pb2

ACTION_TIMEOUT_SECONDS = 20  # timeout for Kinova actions/commands
HOME_POSITION = np.array([0.57665074, 0.00132656, 0.43361586])
# Rectangular workspace boundary for the extended foam table setup.
BOUNDS_LOW = np.array([0.40, -0.36, 0.055])  # meters
BOUNDS_HIGH = np.array([0.95, 0.36, 0.95])   # meters
MAX_POS = np.array([1.5, 0.65, 1.5])  # meters
MAX_POS_EROR = 0.001  # meters


def make_cartesian_action(title, pos, ori_deg):
    action = Base_pb2.Action()
    action.name = title
    action.application_data = ""
    cartesian_pose = action.reach_pose.target_pose
    cartesian_pose.x = pos[0]  # (meters)
    cartesian_pose.y = pos[1]  # (meters)
    cartesian_pose.z = pos[2]  # (meters)
    cartesian_pose.theta_x = ori_deg[0]  # deg
    cartesian_pose.theta_y = ori_deg[1]  # deg
    cartesian_pose.theta_z = ori_deg[2]  # deg
    return action


def execute_action(base, action):
    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_event(e), Base_pb2.NotificationOptions())
    base.ExecuteAction(action)
    done = e.wait(ACTION_TIMEOUT_SECONDS)
    base.Unsubscribe(notification_handle)
    return done


def command_velocity(base, tgt_velocity, duration, velocity_multiplier=0.4):
    """Sends Cartesian velocity command to the Kinova robot.
    Hand-tuned velocity_multiplier is used to get better alignment (0.4 works).
    """
    command = Base_pb2.TwistCommand()
    command.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_BASE
    command.duration = 0  # TODO: figure out what duration does exactly
    twist = command.twist
    twist.linear_x = tgt_velocity[0] * velocity_multiplier
    twist.linear_y = tgt_velocity[1] * velocity_multiplier
    twist.linear_z = tgt_velocity[2] * velocity_multiplier
    twist.angular_x = 0
    twist.angular_y = 0
    twist.angular_z = 0
    finish_time = time.time() + duration
    base.SendTwistCommand(command)
    time.sleep(finish_time - time.time())
    return True


def open_gripper(base):
    """Work in progress (@priyasundaresan)."""
    gripper_command = Base_pb2.GripperCommand()
    finger = gripper_command.gripper.finger.add()
    gripper_command.mode = Base_pb2.GRIPPER_SPEED
    finger.value = 0.05
    base.SendGripperCommand(gripper_command)
    gripper_request = Base_pb2.GripperRequest()
    # Wait for reported position to be opened
    gripper_request.mode = Base_pb2.GRIPPER_POSITION
    while True:
        gripper_measure = base.GetMeasuredGripperMovement(gripper_request)
        if len (gripper_measure.finger):
            print("Current position is : {0}".format(gripper_measure.finger[0].value))
            if gripper_measure.finger[0].value < 0.01:
                break
        else: # Else, no finger present in answer, end loop
            break


def close_gripper(base):
    """Work in progress (@priyasundaresan)."""
    gripper_command = Base_pb2.GripperCommand()
    finger = gripper_command.gripper.finger.add()
    gripper_command.mode = Base_pb2.GRIPPER_SPEED
    finger.value = -0.05
    base.SendGripperCommand(gripper_command)
    gripper_request = Base_pb2.GripperRequest()
    # Wait for reported speed to be 0
    gripper_request.mode = Base_pb2.GRIPPER_SPEED
    while True:
        gripper_measure = base.GetMeasuredGripperMovement(gripper_request)
        if len (gripper_measure.finger):
            print("Current speed is : {0}".format(gripper_measure.finger[0].value))
            if gripper_measure.finger[0].value == 0.0:
                break
        else: # Else, no finger present in answer, end loop
            break


def check_hard_boundary(base, pos):
    """Check that position is in the workspace boundary."""
    # Hardware boundary (meters)
    if (pos < BOUNDS_LOW).any() or (pos > BOUNDS_HIGH).any():
        print('Robot out of bounds: low', BOUNDS_LOW, 'high', BOUNDS_HIGH,
              '\nvs robot pos', pos)
        base.Stop()
        return False
    return True


def check_event(event):
    """Helper for OnNotificationActionTopic that returns END/ABORT function."""
    def check(notification, e=event):
        print("EVENT : " + Base_pb2.ActionEvent.Name(notification.action_event))
        if (notification.action_event == Base_pb2.ACTION_END
            or notification.action_event == Base_pb2.ACTION_ABORT):
            e.set()
    return check


def deg_to_rad(deg):
    rad = deg*np.pi/180
    if np.abs(rad) > np.pi:
        sgn = -1.0 if rad > np.pi else 1.0
        rad += sgn*2*np.pi
    return rad


def get_qpos(base):
    qpos_out = base.GetMeasuredJointAngles()
    qpos_deg = np.zeros(7)
    qpos_rad = np.zeros(7)
    for i in range(len(qpos_deg)):
        qpos_deg[i] = qpos_out.joint_angles[i].value
        qpos_rad[i] = deg_to_rad(qpos_deg[i])
    return qpos_rad, qpos_deg

def get_force(base_cyclic, base):
    base_feedback = base_cyclic.RefreshFeedback()
    fx = base_feedback.base.tool_external_wrench_force_x
    fy = base_feedback.base.tool_external_wrench_force_y
    fz = base_feedback.base.tool_external_wrench_force_z
    tx = base_feedback.base.tool_external_wrench_torque_x
    ty = base_feedback.base.tool_external_wrench_torque_y
    tz = base_feedback.base.tool_external_wrench_torque_z
    
    #x = base_feedback.base.tool_pose_x
    #y = base_feedback.base.tool_pose_y
    #z = base_feedback.base.tool_pose_z
    #kpos = base.GetMeasuredCartesianPose()
    #pos = np.array([kpos.x, kpos.y, kpos.z])
    #print(x,y,z, pos)

    total_force = np.array([fx,fy,fz])
    total_torque = np.array([tx,ty,tz])
    force = np.linalg.norm(total_force)
    torque = np.linalg.norm(total_torque)
    return force,torque

def get_position(base):
    kpos = base.GetMeasuredCartesianPose()
    pos = np.array([kpos.x, kpos.y, kpos.z])
    ori_deg = np.array([kpos.theta_x, kpos.theta_y, kpos.theta_z])
    return pos, ori_deg


def check_position(done, base, tgt_pos):
    """Checks whether the robot reached desired_position."""
    if not done:
        print('Kinova action timeout')
        return False
    pos, _ = get_position(base)
    if not np.allclose(pos, tgt_pos, atol=MAX_POS_EROR):
        print('Kinova did not reach tgt_pos', tgt_pos, 'current pos', pos)
        return False
    return True


def go_pose(base, tgt_pos, tgt_ori_deg, debug=True):
    """Moves Kinova to 3D tgt_pos (meters) and tgt_ori_deg (degrees)."""
    action = make_cartesian_action(
        'goto_position', tgt_pos, tgt_ori_deg)
    if debug:
        print('Starting goto_pose', tgt_pos, tgt_ori_deg)
    done = execute_action(base, action)
    return check_position(done, base, tgt_pos)


def go_home(base):
    """Moves Kinova to a predefined home pose."""
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)
    action_type = Base_pb2.RequestedActionType()
    action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
    action_handle = None
    for action in base.ReadAllActions(action_type).action_list:
        if action.name == "Home":
            action_handle = action.handle
    if action_handle is None:
        print('Failed to reach home pose.')
        print('Failed to reach home pose.')
        return False
    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_event(e), Base_pb2.NotificationOptions())
    base.ExecuteActionFromReference(action_handle)
    done = e.wait(ACTION_TIMEOUT_SECONDS)
    base.Unsubscribe(notification_handle)
    return check_position(done, base, HOME_POSITION)


#
# Kortex utils from
# https://github.com/Kinovarobotics/kortex/blob/master/api_python/examples/
# 103-Gen3_uart_bridge/01-uart_bridge.py
#

class DeviceConnection:
    IP = '192.168.1.10'  # hard-coding IP, since we do not need to change it
    USERNAME = 'admin'
    PASSWORD = 'admin'
    TCP_PORT = 10000
    UDP_PORT = 10001


    @staticmethod
    def createTcpConnection(args=None):
        """
        Returns RouterClient required to create services and send requests
        to device or sub-devices.
        """
        return DeviceConnection(DeviceConnection.IP,
                                port=DeviceConnection.TCP_PORT,
                                credentials=(DeviceConnection.USERNAME,
                                             DeviceConnection.PASSWORD))

    @staticmethod
    def createUdpConnection(args=None): 
        """        
        Returns RouterClient that allows to create services and send requests
        to a device or its sub-devices @ 1khz.
        """
        return DeviceConnection(DeviceConnection.IP, port=DeviceConnection.UDP_PORT,
                                credentials=(DeviceConnection.USERNAME, DeviceConnection.PASSWORD))

    def __init__(self, ipAddress, port=TCP_PORT, credentials = ("","")):

        self.ipAddress = ipAddress
        self.port = port
        self.credentials = credentials
        self.sessionManager = None
        # Setup API
        self.transport = TCPTransport() if port == DeviceConnection.TCP_PORT \
            else UDPTransport()
        self.router = RouterClient(self.transport, RouterClient.basicErrorCallback)

    # Called when entering 'with' statement
    def __enter__(self):
        
        self.transport.connect(self.ipAddress, self.port)

        if (self.credentials[0] != ""):
            session_info = Session_pb2.CreateSessionInfo()
            session_info.username = self.credentials[0]
            session_info.password = self.credentials[1]
            session_info.session_inactivity_timeout = 10000    # (milliseconds)
            session_info.connection_inactivity_timeout = 2000  # (milliseconds)
            self.sessionManager = SessionManager(self.router)
            print("Logging as", self.credentials[0], "on device", self.ipAddress)
            self.sessionManager.CreateSession(session_info)

        return self.router

    # Called when exiting 'with' statement
    def __exit__(self, exc_type, exc_value, traceback):
        if self.sessionManager != None:
            router_options = RouterClientSendOptions()
            router_options.timeout_ms = 1000
            self.sessionManager.CloseSession(router_options)
        self.transport.disconnect()

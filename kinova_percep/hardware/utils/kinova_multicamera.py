"""
Utilities to execute a given trajectory and record images.

@contactrika
[ and @others mentioned in individual function comments ]
"""
import os
import time
import matplotlib.pyplot as plt

import cv2
import numpy as np
import rospy

from scipy.interpolate import griddata, NearestNDInterpolator

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from ..utils.kinova_utils import (
    command_velocity, check_hard_boundary, get_position, get_qpos, get_force)


class Camera:
    def __init__(self, calib_dir, logdir):

        self.logdir = logdir

        self.imgs = []
        self.imgs_depth = []

        dirs = ['video', 'mask', 'depth', 'overlay']
        for dir_name in dirs:
            path = os.path.join(self.logdir, dir_name)
            if not os.path.exists(path):
                os.makedirs(path)

        self.calib_dir = calib_dir
        calib_dir = os.path.join(os.path.dirname(__file__), calib_dir)
        intrinsics_file = os.path.join(calib_dir, 'intrinsics.npy')
        cam2rob_file = os.path.join(calib_dir, 'tcr.npy')

        intrinsics = np.load(intrinsics_file)
        if len(intrinsics) == 4:
            fx,fy,ppx,ppy = intrinsics
            self.K = np.array([[fx,0,ppx],
                               [0,fy,ppy],
                               [0,0,1]])
        else:
            self.K = intrinsics

        self.TCR = np.load(cam2rob_file)

    def get_rgb_depth(self):
        pass

class RealSense(Camera):
    def __init__(self, calib_dir, logdir, rostopic_name, edge_size_px, max_unmasked_z):
        super().__init__(calib_dir, logdir)
        self.camera_timeout = 1.0
        self.rostopic_name = rostopic_name
        self.rgb_image_topic = '/%s/color/image_raw'%(self.rostopic_name)
        self.aligned_depth_image_topic = '/%s/aligned_depth_to_color/image_raw'%(self.rostopic_name)
        print(self.rgb_image_topic, self.aligned_depth_image_topic)
        self.edge_size_px = edge_size_px
        self.max_unmasked_z = max_unmasked_z

    def get_rgb_depth(self):
        msg = rospy.wait_for_message(self.rgb_image_topic, Image,
                                     self.camera_timeout)
        try:
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, -1)  # ROS buffer -> numpy array
        except CvBridgeError as e:
            print('CvBridgeError', e)
            exit(1)

        msg = rospy.wait_for_message(self.aligned_depth_image_topic, Image,
                                     self.camera_timeout)
        try:
            # Need to read in 16bit image, not 8 (hence extra 2nd channel).
            depth_image = np.frombuffer(msg.data, dtype=np.uint16).reshape(
                msg.height, msg.width)
        except CvBridgeError as e:
            print('CvBridgeError', e)
            exit(1)

        self.imgs.append(img)
        self.imgs_depth.append(depth_image)

        return img, depth_image

    def crop_if_needed(self, img):
        if self.edge_size_px > 0:
            img = img[self.edge_size_px:-self.edge_size_px,
                      self.edge_size_px:-self.edge_size_px]
        return img

    def process_raw_rgb(self, img):
        img = self.crop_if_needed(img[:, 80:560])
        return img

    def process_raw_depth(self, depth_image):
        depth_image = depth_image*0.001 # convert meters to mm

        h,w = depth_image.shape
        row_indices = np.arange(h)
        col_indices = np.arange(w)
        pixel_grid = np.meshgrid(col_indices, row_indices)
        pixels = np.c_[pixel_grid[0].flatten(), pixel_grid[1].flatten()].T
        pixels_homog = np.r_[pixels, np.ones([1, pixels.shape[1]])]
        depth_arr = np.tile(depth_image.flatten(), [3, 1])
        points_3d = depth_arr * np.linalg.inv(self.K).dot(pixels_homog)
        points_3d_transf = np.vstack((points_3d, np.ones([1,points_3d.shape[1]])))
        points_3d_transf = (self.TCR.dot(points_3d_transf)).T

        pixels_xyz = points_3d_transf.reshape((h,w,3))

        points_3d_transf_depths = points_3d_transf[:,2]
        depth_image_transf = points_3d_transf_depths.reshape(depth_image.shape)
        depth_image_transf = self.crop_if_needed(depth_image_transf[:, 80:560])

        thresh_idxs = (depth_image_transf < self.max_unmasked_z)
        grayscale_mask = np.ones_like(depth_image_transf)
        grayscale_mask[thresh_idxs] = 0.0
        inpaint_mask = self.crop_if_needed((depth_image > 0)[:,80:560])
        grayscale_mask = grayscale_mask * inpaint_mask
        grayscale_mask *= 255.0

        depth_image_transf = np.clip(depth_image_transf, 0.0, 0.5)
        min_depth, max_depth = np.amin(depth_image_transf), np.amax(depth_image_transf)
        depth_image_norm = (depth_image_transf - min_depth) / (max_depth - min_depth)
        colormap = plt.get_cmap('gist_gray')
        depth_vis = (colormap(depth_image_norm)* 2**8).astype(np.uint8)[:,:,:3]
        depth_vis = cv2.cvtColor(depth_vis, cv2.COLOR_RGB2BGR)

        return grayscale_mask, depth_vis, pixels_xyz

class KinovaMulticamera:
    def __init__(self, logdir, cam_rec_interval=1,
                 edge_size_px=0, max_unmasked_z=0.09):
        # Control config.
        self.control_frequency = 10  # control at 10Hz for now
        self.control_duration = float(1) / self.control_frequency
        # Camera config.
        self.cam_rec_interval = cam_rec_interval
        self.max_unmasked_z = max_unmasked_z

        if not os.path.exists(logdir):
            os.mkdir(logdir)

        self.rootdir = os.path.join(logdir, 'output')
        if not os.path.exists(self.rootdir):
            os.mkdir(self.rootdir)

        rospy.init_node('realsense_stream')
        self.realsense_overhead = RealSense('calib_overhead', os.path.join(self.rootdir, 'overhead'), 'cam_1', edge_size_px, max_unmasked_z)
        self.realsense_side = RealSense('calib_side', os.path.join(self.rootdir, 'side'), 'cam_2', edge_size_px, max_unmasked_z)

        self.cameras = [self.realsense_overhead, self.realsense_side]

    def get_frames(self):
        for c in self.cameras:
            c.get_rgb_depth()

    def save(self):
        for c in self.cameras:
            print('Saving images to', c.logdir)
            for i in range(len(c.imgs)):
                img = c.imgs[i]
                depth_img = c.imgs_depth[i]

                color_img = c.process_raw_rgb(img)
                grayscale_mask, depth_vis, pixels_xyz = c.process_raw_depth(depth_img)

                raw_depth_path = os.path.join(c.logdir, 'depth', f'pixel_xyz_{i:d}.npz')
                np.savez(raw_depth_path, pixels_xyz)

                raw_img_path = os.path.join(c.logdir, 'video', f'img_{i:d}.jpg')
                cv2.imwrite(raw_img_path, color_img)

                depth_path = os.path.join(c.logdir, 'depth', f'depth_{i:d}.jpg')
                cv2.imwrite(depth_path, depth_vis)

    def run_trajectory_record_video(self, base, base_cyclic, tgt_traj, force_thresh=None):
        """Execute tgt_traj (N x 3) with a fixed gripper orientation.
        Record images from the ROS topics set up in constructor.
        """
        pos_traj, pos_traj_for_imgs, qpos_deg_traj_for_imgs, force_torque_traj_for_imgs = [], [], [], []
        ok = True
        for t, step in enumerate(range(len(tgt_traj))):
            curr_pos, _ = get_position(base)
            if not check_hard_boundary(base, curr_pos):  # stops the robot
                ok = False
                break
            pos_traj.append(curr_pos)
            tgt_vel = (tgt_traj[step] - curr_pos) / self.control_duration
            command_velocity(base, tgt_vel, self.control_duration)
            # Record camera images.
            if step % self.cam_rec_interval == 0:
                force, torque = get_force(base_cyclic, base)
                force_torque_traj_for_imgs.append([force, torque])
                pos_traj_for_imgs.append(curr_pos)
                qpos_rad, qpos_deg = get_qpos(base)
                qpos_deg_traj_for_imgs.append(qpos_deg)
                try:
                    self.get_frames()
                except:
                    print('ERROR: RealSense image not streamed.')
                    base.Stop()  # stop the robot
                    exit(1)
                if (force_thresh is not None) and (t>2) and (force > force_thresh):
                    break
        base.Stop()  # stop the robot
        time.sleep(2)  # wait a bit to make sure all buffers are flushed
        if not ok:
            return None
        self.save()
        np.savez(os.path.join(self.rootdir, 'traj_for_imgs.npz'),
                 pos_traj=np.stack(pos_traj_for_imgs),
                 qpos_deg_traj=np.stack(qpos_deg_traj_for_imgs),
                 force_torque_traj=np.stack(force_torque_traj_for_imgs))
        return np.stack(pos_traj)

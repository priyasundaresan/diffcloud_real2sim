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


class KinovaWithCamera:
    def __init__(self, logdir, cam_rec_interval=1,
                 edge_size_px=0, max_unmasked_z=0.09):
        # Control config.
        self.control_frequency = 10  # control at 10Hz for now
        self.camera_timeout = 1.0
        self.control_duration = float(1) / self.control_frequency
        self.time_out_duration = 20
        # Camera config.
        self.cam_rec_interval = cam_rec_interval
        self.edge_size_px = edge_size_px
        self.max_unmasked_z = max_unmasked_z
        rospy.init_node('realsense_stream')
        camera_node_name = 'camera'
        self.rgb_image_topic = '/%s/color/image_raw'%(camera_node_name)
        self.aligned_depth_image_topic = '/%s/aligned_depth_to_color/image_raw'%(camera_node_name)

        # Intrinsics
        #calib_dir = os.path.join(os.path.dirname(__file__), 'calib_side')
        calib_dir = os.path.join(os.path.dirname(__file__), 'calib_overhead')
        intrinsics_file = os.path.join(calib_dir, 'intrinsics.npy')
        cam2rob_file = os.path.join(calib_dir, 'tcr.npy')
        fx,fy,ppx,ppy = np.load(intrinsics_file)
        self.TCR = np.load(cam2rob_file)
        self.K = np.array([[fx,0,ppx], 
                           [0,fy,ppy],
                           [0,0,1]])
        # Logdir setup.
        # tstamp = datetime.strftime(datetime.today(), '%y%m%d_%H%M%S')
        logdir = os.path.join(logdir, 'output')
        self.logdir = logdir
        dirs = ['video', 'mask', 'depth', 'overlay']
        for dir_name in dirs:
            path = os.path.join(self.logdir, dir_name)
            if not os.path.exists(path):
                os.makedirs(path)

    def crop_if_needed(self, img):
        if self.edge_size_px > 0:
            img = img[self.edge_size_px:-self.edge_size_px,
                      self.edge_size_px:-self.edge_size_px]
        return img

    def get_raw_image(self):
        msg = rospy.wait_for_message(self.rgb_image_topic, Image,
                                     self.camera_timeout)
        try:
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, -1)  # ROS buffer -> numpy array
            img = self.crop_if_needed(img[:, 80:560])
            return img
        except CvBridgeError as e:
            print('CvBridgeError', e)
            exit(1)

    def get_depth_mask(self):
        msg = rospy.wait_for_message(self.aligned_depth_image_topic, Image,
                                     self.camera_timeout)
        try:
            # Need to read in 16bit image, not 8 (hence extra 2nd channel).
            depth_image = np.frombuffer(msg.data, dtype=np.uint16).reshape(
                msg.height, msg.width)
            depth_image = depth_image*0.001 # convert meters to mm

            # @Priya, 10/9/21: Note: if storage space is a concern, you
            # can just return depth_image as it is above instead of pixels_xyz.

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
            # Holes in the depth image.
            inpaint_mask = self.crop_if_needed((depth_image > 0)[:,80:560])
            grayscale_mask = grayscale_mask * inpaint_mask
            grayscale_mask *= 255.0 

            # Clipping the depth image for visualization purposes only.
            depth_image_transf = np.clip(depth_image_transf, 0.0, 0.5)
            min_depth, max_depth = np.amin(depth_image_transf), np.amax(depth_image_transf)
            depth_image_norm = (depth_image_transf - min_depth) / (max_depth - min_depth) 
            colormap = plt.get_cmap('gist_gray')
            depth_vis = (colormap(depth_image_norm)* 2**8).astype(np.uint8)[:,:,:3]
            depth_vis = cv2.cvtColor(depth_vis, cv2.COLOR_RGB2BGR)

            return grayscale_mask, depth_vis, pixels_xyz
        except CvBridgeError as e:
            print(e)

    def record_video(self, base, video_length):
        cam_imgs, cam_masks, cam_imgs_depth = [], [], []
        print('Start recording in 10 seconds')
        time.sleep(10)
        for frame_i in range(video_length):
            print(frame_i)
            raw_image, mask, depth = None, None, None
            try:
                raw_image = self.get_raw_image()
                mask, depth, raw_depth = self.get_depth_mask()
            except:
                print('ERROR: RealSense image not streamed.')
                exit(1)
            cam_imgs.append(raw_image)
            cam_masks.append(mask)
            cam_imgs_depth.append(depth)
        print('Stopping robot in 10 seconds')
        time.sleep(10)
        cam_imgs = np.stack(cam_imgs)
        cam_masks = np.stack(cam_masks)
        cam_imgs_depth = np.stack(cam_imgs_depth)
        print('Collected images with shape', cam_imgs.shape)
        for i in range(len(cam_imgs)):
            raw_img_path = os.path.join(self.logdir, 'video', f'img_{i:d}.jpg')
            cv2.imwrite(raw_img_path,
                        cv2.cvtColor(cam_imgs[i], cv2.COLOR_RGB2BGR))
            mask_path = os.path.join(
                self.logdir, 'mask', f'mask_{i:d}.jpg')
            cv2.imwrite(mask_path, cam_masks[i])
            depth_path = os.path.join(
                self.logdir, 'depth', f'depth_{i:d}.jpg')
            cv2.imwrite(depth_path, cam_imgs_depth[i])
        base.Stop()  # stop the robot

    def save(self, imgs, masks, imgs_depth, imgs_raw_depth):
        imgs = np.stack(imgs)
        masks = np.stack(masks)
        imgs_depth = np.stack(imgs_depth)
        imgs_raw_depth = np.stack(imgs_raw_depth)
        alpha = 0.6  # for mask overlay
        print('Saving images with shape', imgs.shape)
        for i in range(len(imgs)):
            # out_img = cv2.cvtColor(cam_imgs[i], cv2.COLOR_RGB2BGR)
            # if self.resize_resolution is not None:
            #    out_img = cv2.resize(out_img, (self.resize_resolution,
            #                                   self.resize_resolution))

            raw_img_path = os.path.join(self.logdir, 'video', f'img_{i:d}.jpg')
            cv2.imwrite(raw_img_path,
                        cv2.cvtColor(imgs[i], cv2.COLOR_RGB2BGR))
            mask_path = os.path.join(
                self.logdir, 'mask', f'mask_{i:d}.jpg')
            cv2.imwrite(mask_path, masks[i])
            depth_path = os.path.join(
                self.logdir, 'depth', f'depth_{i:d}.jpg')
            cv2.imwrite(depth_path, imgs_depth[i])

            #raw_depth_path = os.path.join(
            #    self.logdir, 'depth', f'raw_depth_{i:d}.npy')
            #np.save(raw_depth_path, imgs_raw_depth[i])

            raw_depth_path = os.path.join(
                self.logdir, 'depth', f'pixel_xyz_{i:d}.npz')
            np.savez(raw_depth_path, imgs_raw_depth[i])

            overlay_path = os.path.join(
                self.logdir, 'overlay', f'overlay_{i:d}.jpg')
            mask = masks[i]
            mask_3ch = np.repeat(mask[:,:,np.newaxis], 3, axis=2).astype(np.uint8)
            out_overlay = cv2.addWeighted(imgs[i], alpha, mask_3ch, 1-alpha, 0)
            cv2.imwrite(overlay_path, out_overlay)

    def run_trajectory_record_video(self, base, base_cyclic, tgt_traj, force_thresh=None):
        """Execute tgt_traj (N x 3) with a fixed gripper orientation.
        Record images from the ROS topics set up in constructor.
        """
        pos_traj, pos_traj_for_imgs, qpos_deg_traj_for_imgs, force_torque_traj_for_imgs = [], [], [], []
        cam_imgs, cam_masks, cam_imgs_depth, cam_imgs_raw_depth = [], [], [], []
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
                raw_image, mask, depth, raw_depth = None, None, None, None
                try:
                    raw_image = self.get_raw_image()
                    mask, depth, raw_depth = self.get_depth_mask()
                    raw_image = self.get_raw_image()
                except:
                    print('ERROR: RealSense image not streamed.')
                    base.Stop()  # stop the robot
                    exit(1)
                cam_imgs.append(raw_image)
                cam_masks.append(mask)
                cam_imgs_depth.append(depth)
                cam_imgs_raw_depth.append(raw_depth)
                if (force_thresh is not None) and (t>2) and (force > force_thresh):
                    break
        base.Stop()  # stop the robot
        time.sleep(2)  # wait a bit to make sure all buffers are flushed
        if not ok:
            return None
        self.save(cam_imgs, cam_masks, cam_imgs_depth, cam_imgs_raw_depth)
        np.savez(os.path.join(self.logdir, 'traj_for_imgs.npz'),
                 pos_traj=np.stack(pos_traj_for_imgs),
                 qpos_deg_traj=np.stack(qpos_deg_traj_for_imgs),
                 force_torque_traj=np.stack(force_torque_traj_for_imgs))
        return np.stack(pos_traj)

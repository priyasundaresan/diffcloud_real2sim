#
# MultiCamera class that processes depth and RGB camera input from PyBullet.
# The code follows basic recommendations from PyBullet forums.
# Note that it uses assumptions of the camera setup, which work in the
# current pybullet versions, but ultimately might change in the future.
# Using pybullet versions from 2.6.4 to 2.8.1 should work fine.
#
# @contactrika
#
import os
import sys
import math
import time

import numpy as np
np.set_printoptions(precision=4, linewidth=150, threshold=np.inf, suppress=True)
import pybullet


def assert_close(ars, ars0):
    for ar, ar0 in zip(ars, ars0):
        assert(np.linalg.norm(np.array(ar)-np.array(ar0))<1e-6)


class MultiCamera:
    # In non-GUI mode we will render without X11 context but *with* GPU accel.
    # examples/pybullet/examples/testrender_egl.py
    # Note: use alpha=1 (in rgba), otherwise depth readings are not good
    # Using defaults from PyBullet.
    # See examples/pybullet/examples/pointCloudFromCameraImage.py
    PYBULLET_FAR_PLANE = 10000
    PYBULLET_NEAR_VAL = 0.01
    PYBULLET_FAR_VAL = 1000.0

    @staticmethod
    def init(viz):
        if viz:
            pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 1)
            pybullet.configureDebugVisualizer(
                pybullet.COV_ENABLE_RGB_BUFFER_PREVIEW,1)
            pybullet.configureDebugVisualizer(
                pybullet.COV_ENABLE_DEPTH_BUFFER_PREVIEW,1)
            pybullet.configureDebugVisualizer(
                pybullet.COV_ENABLE_SEGMENTATION_MARK_PREVIEW,1)

    @staticmethod
    def get_cam_vals(cam_rolls, cam_yaws, cam_pitches, cam_dist, cam_target,
                     proj_matrix, fov, aspect_ratio=1.0):
        # load static variables
        near_val = MultiCamera.PYBULLET_NEAR_VAL
        far_val = MultiCamera.PYBULLET_FAR_VAL
        far_plane = MultiCamera.PYBULLET_FAR_PLANE

        # compute cam vals
        cam_vals = []
        for cam_roll, cam_yaw, cam_pitch in zip(cam_rolls, cam_yaws, cam_pitches):
            view_matrix = pybullet.computeViewMatrixFromYawPitchRoll(
                cameraTargetPosition=cam_target, distance=cam_dist,
                yaw=cam_yaw, pitch=cam_pitch, roll=cam_roll, upAxisIndex=2)
            np_view_matrix = np.array(view_matrix).reshape(4, 4)
            if proj_matrix is None:
                proj_matrix = pybullet.computeProjectionMatrixFOV(
                    fov, aspect_ratio, near_val, far_val)
            # print('proj_matrix from computeProjectionMatrixFOV', proj_matrix)
            # res = pybullet.getDebugVisualizerCamera()
            # print('proj_matrix from getDebugVisualizerCamera', res[3])
            # proj_matrix = res[3]
            forward_vec = -np_view_matrix[:3, 2]
            horizon = np_view_matrix[:3, 0] * far_plane * 2 * aspect_ratio
            vertical = np_view_matrix[:3, 1] * far_plane * 2
            cam_vals.append([view_matrix, proj_matrix, forward_vec,
                             horizon, vertical, cam_dist, cam_target])

        return cam_vals

    @staticmethod
    def get_camera_matricies(cam_rolls, cam_yaws, cam_pitches, cam_dist,
                             cam_target, fov, aspect_ratio=1.0):
        view_mat, proj_mat = MultiCamera.get_cam_vals(cam_rolls, cam_yaws,
                                                      cam_pitches, cam_dist,
                                                      cam_target, None,
                                                      fov, aspect_ratio)[0][:2]
        view_mat = np.asarray(view_mat).reshape([4, 4], order='F')
        proj_mat = np.asarray(proj_mat).reshape([4, 4], order='F')
        return view_mat, proj_mat

    @staticmethod
    def render(sim, cam_rolls, cam_yaws, cam_pitches, cam_dist, cam_target,
               proj_matrix, fov=90, width=100, return_seg=False, debug=False):
        imgs, segs = [], []
        cam_vals_lst = MultiCamera.get_cam_vals(
            cam_rolls, cam_yaws, cam_pitches, cam_dist, cam_target,
            proj_matrix, fov)
        for cam_vals in cam_vals_lst:
            view_matrix, proj_matrix, cam_forward, cam_horiz, cam_vert, \
                cam_dist, cam_tgt = cam_vals
            w, h, rgba_px, depth_px, segment_mask = sim.getCameraImage(
                width=width, height=width,
                viewMatrix=view_matrix, projectionMatrix=proj_matrix,
                renderer=pybullet.ER_BULLET_HARDWARE_OPENGL,
                # renderer=pybullet.ER_TINY_RENDERER,
                flags=pybullet.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX)
            rgba_px = rgba_px.astype(float)
            rgba_px[..., -1] = depth_px
            imgs.append(rgba_px)
            segs.append(segment_mask)
        if return_seg:
            return imgs, segs
        else:
            return imgs

import numpy as np
import os
import cv2

class CamRobTransformer:
    def __init__(self):
        calib_dir = os.path.join(os.path.dirname(__file__), 'calib')
        intrinsics_file = os.path.join(calib_dir, 'intrinsics.npy')
        cam2rob_file = os.path.join(calib_dir, 'tcr.npy')
        rob2cam_file = os.path.join(calib_dir, 'trc.npy')
        fx,fy,ppx,ppy = np.load(intrinsics_file)
        self.TCR = np.load(cam2rob_file)
        self.TRC = np.load(rob2cam_file)
        self.K = np.array([[fx,0,ppx], 
                           [0,fy,ppy],
                           [0,0,1]])
        self.resolution_x = 640
        self.resolution_y = 480

    def pixel_to_robxyz(self, depth, pixel):
        point_3d_cam = depth * np.linalg.inv(self.K).dot(np.r_[pixel, 1.0])
        point_3d_rob = (self.TCR.dot(np.r_[point_3d_cam, 1.0])).T
        return point_3d_rob

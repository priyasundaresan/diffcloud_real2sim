import pyrealsense2 as rs
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib as mpl
import pprint
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R

# Create a pipeline
pipeline = rs.pipeline()

# Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)

# Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: " , depth_scale)

# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)

def plot_pointcloud(point_sets, title=""):
    fig = plt.figure(figsize=(5, 5))
    ax = Axes3D(fig)
    for points in point_sets:
        x, y, z = points
        ax.scatter3D(x, y, z, s=50)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.set_title(title)
    plt.show()

def find_chessboard(nrows=5, ncols=5):
    # Streaming loop
    try:
        corners_uv = None
        while corners_uv is None:
            # Get frameset of color and depth
            frames = pipeline.wait_for_frames()
            # frames.get_depth_frame() is a 640x360 depth image
    
            # Align the depth frame to color frame
            aligned_frames = align.process(frames)
    
            # Get aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
            color_frame = aligned_frames.get_color_frame()
    
            # Validate that both frames are valid
            if not aligned_depth_frame or not color_frame:
                continue
    
            depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics

            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
    
            vis = color_image.copy()
    
            grayscale_image = cv2.cvtColor(color_image,cv2.COLOR_BGR2GRAY)
            ret, corners_uv = cv2.findChessboardCorners(grayscale_image, (nrows,ncols), flags=1)
            vis = cv2.drawChessboardCorners(vis, (nrows, ncols), corners_uv, ret)
    
        corners_xyz = []
        intrinsics = np.array([depth_intrin.fx,depth_intrin.fy,depth_intrin.ppx,depth_intrin.ppy])
        for uv in corners_uv:
            print(uv)
            u,v = uv.squeeze()
            depth = aligned_depth_frame.get_distance(u, v)
            print(depth)
            xyz = rs.rs2_deproject_pixel_to_point(depth_intrin, [u, v], depth)
            corners_xyz.append(xyz)

        corners_xyz = corners_xyz[::-1]
        corners_xyz = np.array(corners_xyz).T

        cv2.imshow('Found Corners', vis)
        cv2.waitKey(0)
        plot_pointcloud(corners_xyz)

    finally:
        pipeline.stop()
        return corners_xyz.T, intrinsics

if __name__ == '__main__':
    corners_xyz, intrinsics = find_chessboard()
    print("x range:", np.amax(corners_xyz[:,0]) - np.amin(corners_xyz[:,0]))
    print("y range:", np.amax(corners_xyz[:,1]) - np.amin(corners_xyz[:,1]))
    print("z range:", np.amax(corners_xyz[:,2]) - np.amin(corners_xyz[:,2]))
    print(corners_xyz.shape)
    pprint.pprint(corners_xyz)
    np.save('cam_chessboard.npy', corners_xyz)
    np.save('intrinsics.npy', intrinsics)

import pyrealsense2 as rs
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib as mpl
import pprint
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
from cam_rob_transformer import CamRobTransformer


class PixelSelector:
    def __init__(self):
        pass
    def load_image(self, img):
        self.img = img
    def mouse_callback(self, event, x, y, flags, param):
        cv2.imshow("pixel_selector", self.img)
        if event == cv2.EVENT_LBUTTONDBLCLK:
            self.clicks.append([x, y])
            print(x, y)
            cv2.circle(self.img, (x, y), 3, (255, 255, 0), -1)
    def run(self, img):
        self.load_image(img)
        self.clicks = []
        cv2.namedWindow('pixel_selector')
        cv2.setMouseCallback('pixel_selector', self.mouse_callback)
        while True:
            k = cv2.waitKey(20) & 0xFF
            if k == 27:
                break
        print("clicks", self.clicks)
        return self.clicks

# Create a pipeline
pipeline = rs.pipeline()

# Create a config and configure the pipeline to stream
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

def live_pixel_selection():
    # Streaming loop
    try:
        color_image = None
        while color_image is None:
            # Get frameset of color and depth
            frames = pipeline.wait_for_frames()
    
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
        pixel_selector = PixelSelector()
        pixels = np.array(pixel_selector.run(vis))
        depths = []
        for uv in pixels:
            u,v = uv.squeeze()
            depth = aligned_depth_frame.get_distance(u, v)
            xyz = rs.rs2_deproject_pixel_to_point(depth_intrin, [u, v], depth)
            depths.append(xyz[-1])
        depths = np.array(depths)

    finally:
        pipeline.stop()
        return pixels, depths

if __name__ == '__main__':
    pixels, depths = live_pixel_selection()
    transformer = CamRobTransformer()
    for ((u,v), depth) in zip(pixels, depths):
        rob_xyz = transformer.pixel_to_robxyz(depth, np.array([u,v]))
        print((u,v,depth, rob_xyz))


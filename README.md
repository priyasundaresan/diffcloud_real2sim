# Kinova Manipulation/Perception Basics

[Installation](#install)<br />
[Example Workflow](#workflow)<br />
[Debugging](#debugging)<br />

<a name="install"></a>
## Installation
### Set up conda environment
```
git clone https://github.com/priyasundaresan/kinova_perception
cd kinova_perception
conda create --name kinova_env python=3.8
conda activate kinova_env
conda install -c conda-forge -c iopath fvcore iopath
pip install -e .
# Download whl file from: https://github.com/Kinovarobotics/kortex/blob/master/api_python/examples/readme.md
python3 -m pip install ~/Downloads/kortex_api-2.3.0.post34-py3-none-any.whl 
pip install open3d
```

### Install Blender 2.82
* Visit: https://download.blender.org/release/Blender2.82/ 
* Download the file called `blender-2.82-linux64.tar.xz`
* This will be used for render robot masks from a given trajectory, using knowledge of the Kinova URDF. Thus, in post-processing, the robot can be artifically removed from scenes captured during manipulation.

### Set up Docker container (to install some libraries for post-processing point clouds)
* The library https://github.com/strawlab/python-pcl contains useful tools for processing point clouds captured with the RealSense cameras, but is difficult to install with `pip`, so there is a provided `docker` image for using it. 
```
cd kinova_perception/post_proc/docker
./docker_build.py
# Test that the container built properly using ./docker_run.py (NOTE: use Ctrl+D to exit the container)
```
<a name="workflow"></a>
## Example workflow: recording trajectories
* Add the following lines to your `~/.bashrc.user`, and then run `source ~/.bashrc.user`. Whenever interfacing with the robots/cameras, you'll want to always `source` ROS, and then the other three commands are helpful shortcuts for launching camera ROS nodes and using `blender` on the command line.
```
source /opt/ros/noetic/setup.bash
alias start-topcam='roslaunch realsense2_camera rs_camera.launch camera:=cam_1 serial_no:=919122071583 align_depth:=true initial_reset:=true'
alias start-sidecam='roslaunch realsense2_camera rs_camera.launch camera:=cam_2 serial_no:=838212072814 align_depth:=true initial_reset:=true'
alias blender='/afs/cs.stanford.edu/u/priyasun/Downloads/blender-2.82-linux64/blender'
```
* In one terminal, run `start-topcam` to launch the ROS node for the overhead RealSense camera.
* In another terminal, run `start-sidecam` to launch the ROS node for the side-mounted RealSense camera.
* NOTE: if you swap out either of the cameras, be sure to update the `serial_no` for these commands in the `~/.bashrc.user`. 

* First, run the script to execute a robot trajectory and save RGB and depth images:
```
python -m kinova_percep.hardware.scripts.kinova_cart_vel lift ~/code/tmp
```
* This will create a folder at ~/code/tmp/liftXXXXX that looks like the following. Note, XXXXX is a unique ID from `datetime`:
```
liftXXXXX
└── output
    ├── overhead
    │   ├── depth # contains JPEG depth images, plus pixel_xyz files which record the robot xyz coordinate per-pixel of the depth image
    │   └── video # RGB frames captured during trajectory
    └── side
        ├── depth
        └── video
```
* You can then run various post-processing scripts to extract robot masks and produced final point clouds. First, create a `test_rollouts` directory containing the trajectory
```
cd post_proc
mkdir test_rollouts
mv ~/code/tmp/liftXXXXX ./test_rollouts
```
* From inside the `conda` env, you can run the robot masking script (this masks the robot from RGB video frames, using knowledge of the robot geometry and 3D model). For instance:
```
(kinova_env) priyasun@bohg-ws-13:/juno/u/priyasun/code/kinova_perception/post_proc$ python run_pybullet_blender_masking.py
(kinova_env) priyasun@bohg-ws-13:/juno/u/priyasun/code/kinova_perception/post_proc$ cd docker
(kinova_env) priyasun@bohg-ws-13:/juno/u/priyasun/code/kinova_perception/post_proc$ ./docker_run.py
```
* At this point, you should see the following newly created directories in `test_rollouts/liftXXXXX`:
```
liftXXXXX
└── output
    ├── overhead
    │   ├── depth
    │   ├── mask
    │   ├── mask_dilated # contains masks of robot
    │   ├── overlay
    │   ├── overlay_dilated # contains overlays of robot masks on RGB frames
    │   └── video
    └── side
        ├── depth
        ├── mask
        ├── mask_dilated # contains masks of robot
        ├── overlay
        ├── overlay_dilated # contains overlays of robot masks on RGB frames
        └── video
```
* Then, use the point-cloud processing `docker` container to filter the resulting point clouds, mask out the robot from point clouds, and merge the side/overhead view point clouds (the `-o` and `-s` flags specify the overhead and side directories, respectively).
```
(base) root@b89b9fef4660:/host# python generate_merged_masked_pcls.py -o test_rollouts/liftXXXXX/output/overhead -s test_rollouts/liftXXXXX/output/side
```
#
* The above script creates a directory at `/host/output` which contains the merged, masked point clouds stored as `.npy` files, and renderings of the resulting point clouds.

<a name="install"></a>
## Debugging
### RealSense Cameras
 * When launching `start-topcam` or `start-sidecam`, you may see `WARNINGS` in yellow. These can be safely ignored, but if you see `ERRORS` in red, then there is an issue. 
 * As a first resort, try hardware resets (unplugging and plugging back in the RealSense, plugging in the RealSense to a different USB port)
 * Also, double check that the `serial_no` you're using in `start-sidecam` or `start-topcam` is correct. Verify this in `realsense-viewer` if you are unsure.
 * In general, `realsense-viewer` is a useful commandline tool to launch RealSense visualizations in a GUI. Note, you must kill all camera-related ros nodes before launching `realsense-viewer`, or kill `realsense-viewer` if you are starting the ros nodes (both cannot be running at the same time)
### Kinovas
 * Powering on: Hold down the top button on the back of the Kinova for 3-5 seconds. Once you see a green light turn on, you can release the button. Give it a few more seconds (~5-10), and the light should blink yellow. Then, the Kinova should open --> close --> open its gripper at which point it is on
 * Powering off: Use the XBOX controller to home the robot by holding down `B`. Then, use the right joystick to lower the robot close to the table. Hold down the top button on the back of the Kinova until it loses power. 
 * Note: the Kinova has no brakes, so using E-STOP during execution or not bringing the robot close to the table via XBOX controller before powering off will cause the robot to fall when power is cut. 
 * To stop robot execution while a script is running, use `Ctrl + \`

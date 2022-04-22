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

### Install Blender 2.82 from https://download.blender.org/release/Blender2.82/
This is used for using the URDF of the Kinova robot to render robot masks from the scene captured

### Set up Docker container (to install some libraries for post-processing point clouds)
The library https://github.com/strawlab/python-pcl contains useful tools for processing point clouds captured with the RealSense cameras, but is difficult to install with `pip`, so there is a provided `docker` image for using it. 
```
cd kinova_perception/post_proc/docker
./docker_build.py
# Test that the container built properly using ./docker_run.py (NOTE: use Ctrl+D to exit the container)
```
<a name="workflow"></a>
## Example workflow
* First, run the script to execute a robot trajectory and save RGB and depth images:
```
python -m kinova_percep.hardware.scripts.kinova_cart_vel lift ~/code/tmp
```
* This will create a folder at ~/code/tmp/liftXXXXX that looks like the following. Note, XXXXX is a unique ID from `datetime`:
```
liftXXXXX
└── output
    ├── overhead
    │   ├── depth
    │   ├── mask
    │   ├── overlay
    │   └── video
    └── side
        ├── depth
        ├── mask
        ├── overlay
        └── video
```
* You can then run various post-processing scripts to extract masked point clouds. First, create a `test_rollouts` directory containing the trajectory
```
cd post_proc
mkdir test_rollouts
mv ~/code/tmp/liftXXXXX ./test_rollouts
```
* From inside the `conda` env, you can run the robot masking script:
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
    │   ├── mask_dilated
    │   ├── overlay
    │   ├── overlay_dilated
    │   └── video
    └── side
        ├── depth
        ├── mask
        ├── mask_dilated
        ├── overlay
        ├── overlay_dilated
        └── video
```
# Using docker container
(base) root@b89b9fef4660:/host# python generate_merged_masked_pcls.py -o test_rollouts/liftXXXXX/output/overhead -s test_rollouts/liftXXXXX/output/side
# This will produce a folder output

#
alias start-topcam='roslaunch realsense2_camera rs_camera.launch camera:=cam_1 serial_no:=919122071583 align_depth:=true initial_reset:=true'
alias start-sidecam='roslaunch realsense2_camera rs_camera.launch camera:=cam_2 serial_no:=838212072814 align_depth:=true initial_reset:=true'



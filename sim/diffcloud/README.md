# DiffCloud

[Installation](#install)<br />
[Sim Experiments](#simexps)<br />
[Creating New Simulation Scenarios](#scenarios)<br />

<a name="install"></a>
## Installation
* If not already done, clone this repository:
```
git clone https://github.com/priyasundaresan/diffcloud_real2sim
```
* Install `docker` if not already installed (https://docs.docker.com/engine/install/). This repository is tested with 
```
Docker version 20.10.12, build e91ed57
```
* Install `arcsim` and Python bindings. There is a provided `docker` container to avoid any dependency headaches.
```
cd diffcloud_real2sim/sim/diffcloud/pysim
ln -s ../arcsim/conf ./conf
ln -s ../arcsim/materials ./materials
ln -s ../arcsim/meshes ./meshes
cd ../docker 
./docker_build.py
```
* All development for DiffCloud happens within the provided `docker` container. Also, it is necessary to run `source paths.sh` or `. ./paths.sh` each time you want to do DiffCloud-related things, to source the `conda` environment installed within the container. To test that things are working, try the following and double check that nothing errors:
```
./docker_run.py # run this from /path/to/diffcloud_real2sim/sim/diffcloud/docker
root@96635ca29212:/host# ls
README.md  arcsim  build  docker  paths.sh  pysim
root@96635ca29212:/host# . ./paths.sh
no change     /root/miniconda3/condabin/conda
no change     /root/miniconda3/bin/conda
no change     /root/miniconda3/bin/conda-env
no change     /root/miniconda3/bin/activate
no change     /root/miniconda3/bin/deactivate
no change     /root/miniconda3/etc/profile.d/conda.sh
no change     /root/miniconda3/etc/fish/conf.d/conda.fish
no change     /root/miniconda3/shell/condabin/Conda.psm1
no change     /root/miniconda3/shell/condabin/conda-hook.ps1
no change     /root/miniconda3/lib/python3.9/site-packages/xontrib/conda.xsh
no change     /root/miniconda3/etc/profile.d/conda.csh
modified      /root/.bashrc

==> For changes to take effect, close and re-open your current shell. <==

(diffsim_torch3d) root@96635ca29212:/host# python3
Python 3.6.13 | packaged by conda-forge | (default, Feb 19 2021, 05:36:01) 
[GCC 9.3.0] on linux
Type "help", "copyright", "credits" or "license" for more information.
>>> import torch
>>> import arcsim
>>> import pytorch3d
```
  * NOTE: The above `./docker_run.py` script is intended for if you are doing development on a machine with a display. If you are remotely logged into a machine, you may use `./docker_run_headless.py` but note that scripts like `pysim/msim.py` which launch a visualization will not work.

<a name="simexps"></a>
## Experiments
* Provided are example scripts to run sim-to-sim experiments and real-to-sim experiments. Example target point clouds are provided in `band_sim_pcls`, `hang_sim_pcls`, `lift_real_pcls`, `fold_real_pcls`
* For the following, ensure that you are first inside the Docker container using the above step.
```
(diffsim_torch3d) root@96635ca29212:/host# cd pysim
```
### Sim-to-Sim Experiments
#### Band Stretch
```
(diffsim_torch3d) root@44823e91a9b2:/host/pysim# python run_band_exps.py
# Will produce a folder band_exps_diffcloud containing visualizations
```
#### Vest Hang
```
(diffsim_torch3d) root@44823e91a9b2:/host/pysim# python run_hang_exps.py
# Will produce a folder hang_exps_diffcloud containing visualizations
```
### Real-to-Sim Experiments
* NOTE: These scripts may take a few hours. Each optimization is on the order of minutes but there are ~15 provided trajectories each running for 50 optimization steps.
* To optimize for a single trajectory, for instance, you could run:
```
(diffsim_torch3d) root@44823e91a9b2:/host/pysim# python exp_learn_lift_real2sim.py
# Will produce a folder default_out_exp containing visualizations
```
#### Lift
```
(diffsim_torch3d) root@44823e91a9b2:/host/pysim# python run_lift_exps.py
# Will produce a folder lift_exps_diffcloud containing visualizations
```
#### Fold
```
(diffsim_torch3d) root@44823e91a9b2:/host/pysim# python run_fold_exps.py
# Will produce a folder fold_exps_diffcloud containing visualizations
```
* I like to use the Linux program `eog` to easily look through the visualizations. From outside the `docker` container (`Ctrl + D` to exit), you may use the following, for example, to visualize the learned results. First `sudo apt-get install eog`, then:
```
eog hang_exps_diffcloud/0/fixed* # visualize the optimized result for a single trajectory
eog hang_exps_diffcloud/*/fixed* # visualize the optimized result across trajectories
```
* NOTE: Currently, there is a problem with running the above scripts on A400 GPUs. You can still run the optimizations setting device from `cuda:0` --> `cpu` in the scripts. 

<a name="scenarios"></a>
## Creating New Simulation Scenarios
* Launch `docker` container in non-headless mode: `./docker_run.py` from `diffcloud_real2sim/sim/diffcloud/docker`
* `cd /path/to/diffcloud_real2sim/sim/diffcloud/pysim`
* There is a script called `test_demos.py` showing how to run a provided scenario, tweak simulation deformable parameters manually, and print out simulation trajectories.
```
(diffsim_torch3d) root@44823e91a9b2:/host/pysim# python test_demos.py
```
* Run the following to launch and interactive visualization showing the scenario
```
(diffsim_torch3d) root@44823e91a9b2:/host/pysim# python msim.py
```
* To add your own simulation scenarios, it is a matter of creating the appropriate configuration file in `arcsim/conf` referencing your meshes in `arcsim/meshes`, and adding code like the following to `test_demos.py` + visualizing with `msim.py`:
```
if not os.path.exists('default_out'):
    os.mkdir('default_out')
sim = arcsim.get_sim()
sim_trajectory = []
arcsim.init_physics(os.path.join('conf/rigidcloth/PATH_TO_YOUR_JSON'),'default_out/out0', False)
for step in range(YOUR_N_STEPS):
    sim_trajectory.append(sim.cloths[0].mesh.nodes[YOUR_PINNED_VERTEX].x.detach().cpu().numpy()) 
    print(step, np.round(sim_trajectory[-1], 4))
    arcsim.sim_step()
print('saving')
np.save('sim_traj.npy', np.array(sim_trajectory))
```
* There is also a script `render_pcl_demo.py` to convert the result of meshes saved in `default_out` produced by `msim.py` to point clouds in a folder called `demo_pcl_frames`. An example workflow:
```
(diffsim_torch3d) root@44823e91a9b2:/host/pysim# python test_demos.py
(diffsim_torch3d) root@44823e91a9b2:/host/pysim# python msim.py
(diffsim_torch3d) root@44823e91a9b2:/host/pysim# python render_pcl_demo.py # saves to output folder demo_pcl_frames
```
* The folder `demo_pcl_frames` can be used to overlay against real point clouds recorded, see [the bottom of this section](https://github.com/priyasundaresan/diffcloud_real2sim/tree/master/real#example-of-data-collection-recording-trajectories-and-paired-point-clouds)

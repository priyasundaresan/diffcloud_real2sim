# DiffCloud

[Installation](#install)<br />
[Sim Experiments](#simexps)<br />

<a name="install"></a>
## Installation
* If not already done, clone this repository:
```
git clone https://github.com/priyasundaresan/diffcloud_real2sim
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
./docker_run.py
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
## Sim Experiments


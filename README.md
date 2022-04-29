# DiffCloud: Deformable Real-to-sim from Point Clouds

*Priya Sundaresan, Rika Antonova, Jeannette Bohg*

[[Project]](tinyurl.com/diffcloud)
[[arXiv]](https://arxiv.org/abs/2204.03139)

## Description
* This repo provides a framework for differentiable real-to-sim parameter estimation of deformable objects from real robot manipulation.
* It is organized into
  * `real`: scripts for interfacing with a real Kinova robot + RealSense cameras for executing trajectories and recording point clouds
  * `sim`: scripts for performing real-to-sim optimizations with differentiable and nondifferentiable methods.
* Please see the details of our DiffCloud implementation [here](https://github.com/priyasundaresan/diffcloud_real2sim/tree/master/sim/diffcloud)
* Note, this is heavily built on [DiffSim] (https://github.com/YilingQiao/diffsim)

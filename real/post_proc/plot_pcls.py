'''
python plot_pcls.py -r output -s fling_pcl_frames_shorter -t yaml_configs/fling_real2sim.yaml
'''

import matplotlib.pyplot as plt
import pprint
from mpl_toolkits.mplot3d import Axes3D
import os
import numpy as np
from scipy.spatial.transform import Rotation as R
import cv2
import argparse
import json
import yaml

def plot_pointclouds(pcls, title="", angle=0):
    fig = plt.figure(figsize=(10, 5))
    ax = fig.add_subplot(1, 1, 1, projection='3d')
    for i,points in enumerate(pcls):
        x, y, z = points.T
        if i==1:
            ax.scatter3D(x, y, z, s=0.2, c='r')
        else:
            ax.scatter3D(x, y, z, s=0.2)
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')

        ax.set_xlim([-1.0,1.0])
        ax.set_ylim([-1.0,1.0])
        ax.set_zlim([0,1.5])

        ax.view_init(10, angle)

    #plt.show()
    plt.savefig(title)
    plt.clf()
    plt.cla()
    plt.close(fig)

def traj_to_motion(traj, frame_time, skip=2):
    data = {"motions": [{"time": 0}]}
    #for i in range(1,len(traj),skip):
    for i in range(len(traj),skip):
        waypt = traj[i]
        val_i = {"time": (i+1)*frame_time, "transform": {"translate": list(waypt), "scale": 0}}
        data["motions"].append(val_i)
    data["motions"] = [data["motions"]]
    with open('motion.json', 'w') as f:
        json.dump(data, f, indent=4)
    pprint.pprint(data)

def traj_to_vel(traj):
    traj_t1 = traj[1:]
    dt = 0.01
    vels = (traj_t1 - traj[:-1])/dt
    np.save('vels.npy', vels)

def real2sim_transf(points, trans, scale, rot_inv, compensation_factor=None):
    if compensation_factor is not None:
        points += compensation_factor
    return ((points - trans)/scale)@rot_inv

def sim2real_transf(points, trans, scale, rot):
    return points@rot*scale + trans

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-r', '--real_dir', default='output')
    parser.add_argument('-s', '--sim_dir', default='demo_pcl_frames')
    parser.add_argument('-t', '--transform_yaml_config', type=str, required=False)
    args = parser.parse_args()

    real_dir = args.real_dir
    sim_dir = args.sim_dir

    output_dir = 'pcls'
    if not (os.path.exists(output_dir)):
        os.mkdir(output_dir)
        
    if args.transform_yaml_config is not None:
        with open(args.transform_yaml_config) as stream:
            config = yaml.safe_load(stream)
        rot = R.from_euler('z', config['z_rotation'], degrees=True).as_matrix()
        rot_inv = R.from_euler('z', -config['z_rotation'], degrees=True).as_matrix()
        trans = np.array(config['translation'])
        scale = config['scale']
        compensation_factor = np.array(config['compensation_factor'])

    episode_length = len(os.listdir(real_dir))
    # loop
    delay = 0

    for i in range(delay,episode_length):
        #pcl1 = np.load(os.path.join(real_dir, '%03d.npy'%i))
        pcl1 = np.load(os.path.join(real_dir, '%03d.npy'%(i-delay)))
        if args.transform_yaml_config is not None:
            pcl1 = real2sim_transf(pcl1, trans, scale, rot, compensation_factor=compensation_factor)

        pcl2 = np.load(os.path.join(sim_dir, '%03d.npy'%(i))).squeeze()

        plot_pointclouds([pcl1, pcl2], title=os.path.join(output_dir, 'rot_%03d.jpg'%i), angle=(i*185/episode_length))
        plot_pointclouds([pcl1, pcl2], title=os.path.join(output_dir, 'fixed_%03d.jpg'%i), angle=0)
        print(i)

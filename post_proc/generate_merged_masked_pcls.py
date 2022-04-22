'''
cd docker
./docker_run.py
python generate_merged_masked_pcls.py -o fling_rollouts/fling_tan2/output/overhead -s fling_rollouts/fling_paper2/output/side -n 2500 -t yaml_configs/fling_real2sim.yaml
# Ctrl + D
'''

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
import numpy as np
import cv2
import argparse
import pcl
from scipy import interpolate
from scipy.spatial.transform import Rotation as R
from plot_pcls import real2sim_transf, sim2real_transf
import yaml

def plot_pointclouds(pcls, title="", frame='real'):
    fig = plt.figure(figsize=(10, 5))
    titles=['curr', 'ref']
    ax = fig.add_subplot(1, 2, 1, projection='3d')
    for i,points in enumerate(pcls):
        #ax = fig.add_subplot(1, 2, i+1, projection='3d')
        x, y, z = points.T
        ax.set_box_aspect((1,1,1))
        ax.scatter3D(x, y, z, s=0.05)
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
        if frame=='real':
            ax.set_xlim([0.4,1.0])
            ax.set_ylim([-0.3,0.3])
            ax.set_zlim([0.0,0.6])
        elif frame=='sim':
            ax.set_xlim([-1.0,1.0])
            ax.set_ylim([-1.0,1.0])
            ax.set_zlim([0,1.5])
        ax.view_init(10, 0)
        ax.set_title(titles[i])
    #plt.show()
    plt.savefig(title)
    plt.clf()
    plt.cla()
    plt.close(fig)

def downsample_pointcloud(xyz, n_points):
    random_idxs = np.random.choice(np.arange(len(xyz)), n_points)
    downsampled_xyz = xyz[random_idxs]
    return downsampled_xyz

def filter_pointcloud(xyz):
    p = pcl.PointCloud(xyz.astype(np.float32))
    clipper = p.make_cropbox()
    tx = 0
    ty = 0
    tz = 0
    clipper.set_Translation(tx, ty, tz)
    rx = 0
    ry = 0
    rz = 0
    clipper.set_Rotation(rx, ry, rz)

    # lift
    minx = 0.5
    minz = 0.075
    miny = -0.35
    maxx = 0.95
    maxy = 0.35
    maxz = 0.35
    mins = 0
    maxs = 0

    ## fold
    #minx = 0.5
    #minz = 0.085
    #miny = -0.35
    #maxx = 0.95
    #maxy = 0.35
    #maxz = 0.35
    #mins = 0
    #maxs = 0


    clipper.set_MinMax(minx, miny, minz, mins, maxx, maxy, maxz, maxs)
    p = clipper.filter()

    fil = p.make_statistical_outlier_filter()
    fil.set_mean_k(175)
    fil.set_std_dev_mul_thresh(0.5)
    result = fil.filter().to_array()
    return result

def inpaint_depth(pixel_xyz, inpaint_value):
    depth = pixel_xyz[:,:,2]
    valid_mask = np.where(np.around(depth,3) == inpaint_value)
    valid_mask = ~(np.around(depth,3) == inpaint_value)
    coords = np.array(np.nonzero(valid_mask)).T
    values = depth[valid_mask]
    it = interpolate.LinearNDInterpolator(coords, values, fill_value=0)
    filled = it(list(np.ndindex(depth.shape))).reshape(depth.shape)
    pixel_xyz[:,:,2] = filled
    return pixel_xyz

def crop_to_size(inp, target_size):
    dim1 = inp.shape[0]
    dim1_target = target_size[0]
    dim2 = inp.shape[1]
    dim2_target = target_size[1]
    if dim1 != dim1_target:
        crop_amt = (dim1 - dim1_target)//2
        inp = inp[crop_amt:dim1 - crop_amt]
    if dim2 != dim2_target:
        crop_amt = (dim2 - dim2_target)//2
        inp = inp[:, crop_amt:dim2 - crop_amt]
    return inp

def process(mask_dir, depth_dir, idx, n_points):
    mask = cv2.imread(os.path.join(mask_dir, 'mask_%d.jpg'%idx), 0)
    _, mask = cv2.threshold(mask, 127,255,cv2.THRESH_BINARY)
    pixel_xyz = np.load(os.path.join(depth_dir, 'pixel_xyz_%d.npz'%idx))['arr_0']
    pixel_xyz = crop_to_size(pixel_xyz, mask.shape)
    
    points_unmasked  = pixel_xyz[mask == 0]

    pcl = points_unmasked
    pcl = filter_pointcloud(pcl)
    return pcl

def main(mask_dirs, depth_dirs, translations, output_dir, n_points, transform_yaml_config=None):
    for i in range(len(os.listdir(mask_dirs[0]))):
        pcl_combined = None
        pcls_plotting = []
        for mask_dir, depth_dir, translation in zip(mask_dirs, depth_dirs, translations):
            pcl = process(mask_dir, depth_dir, i, n_points)
            pcl += translation
            pcls_plotting.append(pcl)
            pcl_combined = pcl if pcl_combined is None else np.vstack((pcl_combined, pcl))

        print(pcl_combined.shape)
        pcl_combined = downsample_pointcloud(pcl_combined, n_points)
        print(i, pcl_combined.shape)

        if transform_yaml_config is not None:
            with open(transform_yaml_config) as stream:
                config = yaml.safe_load(stream)
            rot = R.from_euler('z', config['z_rotation'], degrees=True).as_matrix()
            rot_inv = R.from_euler('z', -config['z_rotation'], degrees=True).as_matrix()
            trans = np.array(config['translation'])
            scale = config['scale']
            compensation_factor = np.array(config['compensation_factor'])
            pcl_combined = real2sim_transf(pcl_combined, trans, scale, rot, compensation_factor=compensation_factor)

        plot_pointclouds(pcls_plotting, title='%s/%03d.jpg'%(output_dir, i))
        np.save('%s/%03d.npy'%(output_dir, i), pcl_combined)

if __name__ == '__main__':
    # HARDCODED OFFSETS TO MERGE POINTCLOUDS (DUE TO SENSOR/CALIBRATION ERROR)
    translation_overhead_rs = np.array([0,0,0]) # keep this zero, update the others
    translation_side_rs = np.array([0.0,0.0,0.0]) # realsense, side
    translation_phoxi = np.array([0.0,0.0,0.0]) # phoxi, side (from packard)

    parser = argparse.ArgumentParser()
    parser.add_argument('-o', '--realsense_overhead_dir', type=str, required=False)
    parser.add_argument('-p', '--phoxi_dir', type=str, required=False)
    parser.add_argument('-s', '--realsense_side_dir', type=str, required=False)
    parser.add_argument('-n', '--num_points', type=int, default=2500)
    parser.add_argument('-t', '--transform_yaml_config', type=str, required=False)
    args = parser.parse_args()

    mask_dirs = []
    depth_dirs = []
    translations = []

    for input_dir, trans in zip([args.realsense_overhead_dir, args.realsense_side_dir, args.phoxi_dir], [translation_overhead_rs, translation_side_rs, translation_phoxi]):
        if input_dir is not None:
            mask_dirs.append(os.path.join(input_dir, 'mask_dilated'))
            depth_dirs.append(os.path.join(input_dir, 'depth'))
            translations.append(trans)

    output_dir = 'output'
    if not os.path.exists(output_dir):
        os.mkdir(output_dir)

    if args.transform_yaml_config is not None:
        os.system('cp %s %s/'%(args.transform_yaml_config, output_dir))

    main(mask_dirs, depth_dirs, translations, output_dir, args.num_points, transform_yaml_config=args.transform_yaml_config)

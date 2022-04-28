import matplotlib.pyplot as plt
import numpy as np
import os
import open3d as o3d

def visualize(pcls, colors):
    pcds = []
    for pcl, color in zip(pcls, colors):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pcl)
        pcd.paint_uniform_color(color)
        pcds.append(pcd)
    o3d.visualization.draw_geometries(pcds)

if __name__ == '__main__':
    for idx in range(25):
        overhead_cloud = np.load('output/%03d_overhead.npy'%(idx))
        offset = np.array([0,0.0,0.0])
        side_cloud = np.load('output/%03d_side.npy'%idx) + offset
        visualize([overhead_cloud, side_cloud], [[0,0.5,0], [0.5,0,0]]) # overhead is green, side is red


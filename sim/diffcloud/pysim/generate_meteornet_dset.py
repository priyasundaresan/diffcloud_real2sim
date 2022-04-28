import torch
import pprint
import arcsim
import gc
import time
import json
import sys
import gc
import os
import numpy as np
import random
import argparse

import torch.nn as nn
import torch.nn.functional as F
from datetime import datetime

import pytorch3d
from pytorch3d.structures import Meshes
from pytorch3d.io import load_obj
from pytorch3d.ops import sample_points_from_meshes
from pytorch3d.loss import (
    chamfer_distance, 
)
from pytorch3d.transforms import RotateAxisAngle

import matplotlib
matplotlib.use('Agg')
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib as mpl

os.environ['CUDA_VISIBLE_DEVICES'] = '0'

def run_sim(conf_file, demo_length):
    if not os.path.exists('default_out'):
        os.mkdir('default_out')

    sim = arcsim.get_sim()
    arcsim.init_physics(os.path.join(conf_file),'default_out/out0', False)

    stretch = sim.cloths[0].materials[0].stretchingori
    drag = sim.wind.drag

    if 'fling' in conf_file:
        task = 'fling'
    elif 'band' in conf_file:
        task = 'band'
    elif 'stretch' in conf_file:
        task = 'stretch'
    elif 'fold' in conf_file:
        task = 'fold'
    elif 'lift' in conf_file:
        task = 'lift'
    elif 'lasso' in conf_file:
        task = 'lasso'
    elif 'mask' in conf_file:
        task = 'mask'
    elif 'hang' in conf_file:
        task = 'hang'

    mass_mult = random.choice((np.random.uniform(0.1, 1.0, 1), np.random.uniform(1, 3, 1)))[0]
    if task == 'stretch':
        stretch_mult = random.choice((np.random.uniform(0.1, 1.0, 1), np.random.uniform(0.1, 20, 1)))[0]
        drag_mult = 0
        params = np.array([stretch_mult, mass_mult])
    elif task == 'fling':
        stretch_mult = random.choice((np.random.uniform(0.5, 1.0, 1), np.random.uniform(0.5, 20, 1)))[0]
        drag_mult = random.choice((np.random.uniform(0, 2, 1), np.random.uniform(1, 35, 1)))[0]
        params = np.array([stretch_mult, mass_mult, drag_mult])
    elif task == 'fold':
        mode = random.choice(('soft', 'medium', 'stiff', 'random'))
        if mode == 'soft':
            stretch_mult = np.random.uniform(0.05, 1.0, 1)[0]
            mass_mult = np.random.uniform(5, 10, 1)[0]
        elif mode == 'medium':
            stretch_mult = np.random.uniform(1, 10, 1)[0]
            mass_mult = np.random.uniform(1, 5, 1)[0]
        elif mode == 'stiff':
            stretch_mult = np.random.uniform(1, 10, 1)[0]
            mass_mult = np.random.uniform(0.1, 1, 1)[0]
        else:
            stretch_mult = random.choice((np.random.uniform(0.05, 1.0, 1), np.random.uniform(0.5, 10, 1)))[0]
            mass_mult = random.choice((np.random.uniform(0.1, 1.0, 1), np.random.uniform(1, 10, 1)))[0]
        drag_mult = 0
        params = np.array([stretch_mult, mass_mult])
    elif task == 'lift':
        mode = random.choice(('soft', 'medium', 'stiff', 'random'))
        if mode == 'soft':
            stretch_mult = np.random.uniform(0.1, 1.0, 1)[0]
            mass_mult = np.random.uniform(0.1, 1.9, 1)[0]
        elif mode == 'medium':
            stretch_mult = np.random.uniform(0.75, 1.25, 1)[0]
            mass_mult = np.random.uniform(1, 20, 1)[0]
        elif mode == 'stiff':
            stretch_mult = np.random.uniform(1, 20, 1)[0]
            mass_mult = np.random.uniform(0.1, 1, 1)[0]
        else:
            stretch_mult = random.choice((np.random.uniform(0.5, 1.0, 1), np.random.uniform(0.5, 10, 1)))[0]
            mass_mult = random.choice((np.random.uniform(0.1, 1.0, 1), np.random.uniform(1, 20, 1)))[0]
        drag_mult = 0
        params = np.array([stretch_mult, mass_mult])
    elif task == 'lasso' or task == 'hang' or task == 'band':
        mode = random.choice(('soft', 'medium', 'stiff', 'random'))
        if mode == 'soft':
            stretch_mult = np.random.uniform(0.1, 1.0, 1)[0]
            mass_mult = np.random.uniform(5, 10, 1)[0]
            print('here', stretch_mult, mass_mult)
        elif mode == 'medium':
            stretch_mult = np.random.uniform(1, 10, 1)[0]
            mass_mult = np.random.uniform(1, 5, 1)[0]
        else:
            stretch_mult = np.random.uniform(1, 10, 1)[0]
            mass_mult = np.random.uniform(0.1, 1, 1)[0]
        drag_mult = 0
        params = np.array([stretch_mult, mass_mult])
    elif task == 'mask':
        mode = random.choice(('soft', 'medium', 'stiff', 'random'))
        if mode == 'soft':
            stretch_mult = np.random.uniform(0.7, 1.0, 1)[0]
            mass_mult = np.random.uniform(2, 4, 1)[0]
        elif mode == 'medium':
            stretch_mult = np.random.uniform(1, 10, 1)[0]
            mass_mult = np.random.uniform(1, 4, 1)[0]
        else:
            stretch_mult = np.random.uniform(1, 10, 1)[0]
            mass_mult = np.random.uniform(0.1, 1, 1)[0]
        drag_mult = 0
        params = np.array([stretch_mult, mass_mult])

    sim.cloths[0].materials[0].stretchingori = stretch*stretch_mult
    arcsim.reload_material_data(sim.cloths[0].materials[0])
    sim.wind.drag = drag*drag_mult

    print(params)
    if task == 'lift':
        corner_idxs = [5,9,10,11,17,21,22,23,27,28,33,35,36,37,39,43,45,46,47,48]
        for i,node in enumerate(sim.cloths[0].mesh.nodes):
            if i in corner_idxs:
                node.m  *= mass_mult
    elif task == 'fold':
        corner_idxs = [9,10,11,21,22,36,37,43,45,46,47,48]
        for i,node in enumerate(sim.cloths[0].mesh.nodes):
            if i in corner_idxs:
                node.m  *= mass_mult
    elif task == 'hang':
        corner_idxs = [38,39,40,42,43,47,49,51,54,55,56,57,58,59,60,61,62,63,64,65,67,68]
        for i,node in enumerate(sim.cloths[0].mesh.nodes):
            if i in corner_idxs:
                node.m  *= mass_mult
    else:
        for node in sim.cloths[0].mesh.nodes:
            node.m  *= mass_mult

    for step in range(demo_length):
        print(step)
        arcsim.sim_step()
    return params

def sim_objs_to_pcls(num_points, demo_length, offset=0, step=1, sim_dir='default_out', deformable_dropout=0.7, noise_injection=True, render_obstacles=False, gradual_dropout=False, faces_idx=None):
    sample_pcls = []
    for i in range(offset, demo_length+offset, step):
        mesh_fnames = sorted([f for f in os.listdir('%s/out0'%sim_dir) if '%04d'%i in f])
        all_verts = []
        all_faces = []
        vert_count = 0
        #fnames = mesh_fnames[:1] + mesh_fnames [2:] if render_obstacles else mesh_fnames[:1]
        #fnames = mesh_fnames[:-1] if render_obstacles else mesh_fnames[:1]
        fnames = mesh_fnames if render_obstacles else mesh_fnames[:-1]
        if gradual_dropout:
            dropout_amt = max(0.05, (i-offset)/(demo_length)*deformable_dropout)
        else:
            dropout_amt = np.random.uniform(deformable_dropout, 1.0)
        for j, f in enumerate(fnames):
            verts, faces, _ = load_obj(os.path.join(sim_dir, "out0", f))
            if noise_injection:
                verts += torch.randn(len(verts), 3)*np.random.uniform(0,0.035)
                if j > 0: # dropout obstacle
                    sampled_idxs = torch.from_numpy(np.random.choice(len(faces.verts_idx), int(np.random.uniform(0.3, 0.9)*len(faces.verts_idx)), replace=False))
                    faces_idx = list(range(len(faces.verts_idx)))
                else: #  dropout deformable
                    # fold
                    faces_idx = list(range(len(faces.verts_idx))) if faces_idx is None else faces_idx
                    sampled_idxs = torch.from_numpy(np.random.choice(faces_idx, int(dropout_amt*len(faces_idx)), replace=False))
                    # everything else
                    #sampled_idxs = torch.from_numpy(np.random.choice(len(faces.verts_idx), int(dropout_amt*len(faces.verts_idx)), replace=False))
                faces = faces.verts_idx[sampled_idxs] + vert_count
            else:
                faces_idx = list(range(len(faces.verts_idx))) 
                #sampled_idxs = torch.from_numpy(np.random.choice(faces_idx, len(faces_idx), replace=False))
                #faces = faces.verts_idx[sampled_idxs] + vert_count
                faces = faces.verts_idx[faces_idx] + vert_count
            vert_count += len(verts)
            all_verts.append(verts)
            all_faces.append(faces)
        mesh = Meshes(verts=[torch.cat(all_verts)], faces=[torch.cat(all_faces)])
        sample_pcl = sample_points_from_meshes(mesh, num_points).squeeze().numpy()
        sample_pcls.append(sample_pcl)
    return np.array(sample_pcls)

def plot_pointcloud(points, title=""):
    x, y, z = points
    fig = plt.figure(figsize=(5, 5))
    ax = Axes3D(fig)
    ax.scatter3D(x, y, z, s=0.2)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.set_xlim([-1,1])
    ax.set_ylim([-1,1])
    ax.set_zlim([0,2])
    ax.view_init(30, 40)
    plt.savefig(title)

def generate_dset(num_episodes, num_points, conf_file, demo_length, noise_injection, deformable_dropout, render_obstacles, gradual_dropout):
    output_dir = 'processed_dset'
    if not (os.path.exists(output_dir)):
        os.mkdir(output_dir)
    for episode in range(num_episodes):
        print(episode)
        params = run_sim(conf_file, demo_length)
        if 'fold' in conf_file:
            faces_idx = [1,2,8,9,10,11,12,13,16,19,22,23,24,25,26,27,28,29,30,34,35,37,38,44,45,46,47,48,49,52,55,58,59,60,61,62,63,64,65,66,70,71]
        else:
            faces_idx = None
        pcl_video = sim_objs_to_pcls(num_points, demo_length, deformable_dropout=deformable_dropout, \
                                                              noise_injection=noise_injection, \
                                                              render_obstacles=render_obstacles, \
                                                              gradual_dropout=gradual_dropout, \
                                                              faces_idx = faces_idx)
        for i in range(demo_length):
            plot_pointcloud(pcl_video[i].T, title=os.path.join(output_dir, '%02d_%03d.jpg'%(episode,i)))
        np.savez_compressed(os.path.join(output_dir, '%03d.npz'%episode), pcl_video=pcl_video, params=params)

def generate_dset_eval(num_episodes, num_points, conf_file, demo_length, noise_injection, deformable_dropout, render_obstacles, gradual_dropout):
    output_dir = 'test_pcls'
    if not (os.path.exists(output_dir)):
        os.mkdir(output_dir)
    gt_params = []
    for episode in range(num_episodes):
        os.mkdir(os.path.join(output_dir, str(episode)))
        params = run_sim(conf_file, demo_length)
        gt_params.append(params)
        if 'fold' in conf_file:
            faces_idx = [1,2,8,9,10,11,12,13,16,19,22,23,24,25,26,27,28,29,30,34,35,37,38,44,45,46,47,48,49,52,55,58,59,60,61,62,63,64,65,66,70,71]
        else:
            faces_idx = None
        pcl_video = sim_objs_to_pcls(num_points, demo_length, deformable_dropout=deformable_dropout, \
                                                              noise_injection=noise_injection, \
                                                              render_obstacles=render_obstacles, \
                                                              gradual_dropout=gradual_dropout, \
                                                              faces_idx = faces_idx)
        for i in range(demo_length):
            np.save(os.path.join(output_dir, str(episode), '%03d.npy'%i), pcl_video[i])
    np.save('%s/gt.npy'%(output_dir), dict(zip([str(e) for e in range(num_episodes)], gt_params)))

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-e', '--num_episodes', type=int)
    parser.add_argument('-p', '--num_points', type=int)
    parser.add_argument('-c', '--conf_file', type=str)
    parser.add_argument('-l', '--demo_length', type=int)
    parser.add_argument('-n', '--noise_injection', action='store_true')
    parser.add_argument('-d', '--deformable_dropout', type=float, default=0.7)
    parser.add_argument('-o', '--render_obstacles', action='store_true')
    parser.add_argument('-g', '--gradual_dropout', action='store_true')
    parser.add_argument('--eval', action='store_true')
    args = parser.parse_args()
    if args.eval:
        print('here')
        generate_dset_eval(args.num_episodes, args.num_points, args.conf_file, args.demo_length, args.noise_injection, args.deformable_dropout, args.render_obstacles, args.gradual_dropout)
    else:
        generate_dset(args.num_episodes, args.num_points, args.conf_file, args.demo_length, args.noise_injection, args.deformable_dropout, args.render_obstacles, args.gradual_dropout)

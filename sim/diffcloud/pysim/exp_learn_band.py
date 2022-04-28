import torch
import pprint
import torch.nn as nn
import torch.nn.functional as F
import arcsim
import gc
import time
import json
import sys
import gc
import numpy as np
import os
from datetime import datetime

import argparse

import pytorch3d
from pytorch3d.structures import Meshes
from pytorch3d.io import load_obj
from pytorch3d.ops import sample_points_from_meshes
from pytorch3d.loss import (
    chamfer_distance, 
)

from mpl_toolkits.mplot3d import Axes3D
import matplotlib
matplotlib.use('agg')
import matplotlib.pyplot as plt
import matplotlib as mpl
from scipy.spatial.transform import Rotation as R
from utils.chamfer import chamfer_distance_one_sided

low_stretch = 0.1
high_stretch = 10
low_mass = 0.1
high_mass = 10

device = torch.device("cuda:0")

print(sys.argv)
out_path = 'default_out_exp'
if not os.path.exists(out_path):
    os.mkdir(out_path)

timestamp = datetime.now().strftime('%Y-%m-%d_%H:%M:%S')

def scale(x, lower_bound, upper_bound, inverse=False):
    if not inverse:
        return lower_bound + x*(upper_bound-lower_bound)
    else:
        return (x-lower_bound)/(upper_bound-lower_bound)

with open('conf/rigidcloth/band_stretch/demo.json','r') as f:
	config = json.load(f)

def save_config(config, file):
	with open(file,'w') as f:
		json.dump(config, f)

save_config(config, out_path+'/conf.json')

torch.set_num_threads(8)
spf = config['frame_steps']
total_steps = 25
#num_points = 3500
num_points = 5500

scalev=1

def reset_sim(sim, epoch):
    arcsim.init_physics(out_path+'/conf.json', out_path+'/out%d'%epoch,False)

def plot_pointclouds(pcls, title="", angle=0):
    fig = plt.figure(figsize=(30, 5))
    titles=['curr', 'ref']

    ax_both = fig.add_subplot(141, projection='3d')
    ax_curr = fig.add_subplot(143, projection='3d')
    ax_ref = fig.add_subplot(144, projection='3d')
    ax_initial = fig.add_subplot(142, projection='3d')
    colors = ['deepskyblue', 'green', 'black']
    labels = ['curr', 'ref', 'initial']
    for i,(color,label,points) in enumerate(zip(colors,labels,pcls)):

        x, y, z = points.detach().cpu().numpy().T
        ax_both.scatter3D(x, y, z, s=0.2, label=label, color=color)
        if i==0:
            ax_curr.scatter3D(x, y, z, s=0.2, label=label, color=color)
        elif i==1:
            ax_ref.scatter3D(x, y, z, s=0.2, label=label, color=color)
        else:
            ax_initial.scatter3D(x, y, z, s=0.2, label=label, color=color)
        ax_both.set_xlabel('x')
        ax_both.set_ylabel('y')
        ax_both.set_zlabel('z')

        xmin,xmax = -1.0,1.0
        ymin,ymax = -1.0,1.0
        zmin,zmax = 0,1.5

        # this is for sim frame
        ax_both.set_xlim([xmin,xmax])
        ax_both.set_ylim([ymin,ymax])
        ax_both.set_zlim([zmin,zmax])

        ax_curr.set_xlim([xmin,xmax])
        ax_curr.set_ylim([ymin,ymax])
        ax_curr.set_zlim([zmin,zmax])

        ax_ref.set_xlim([xmin,xmax])
        ax_ref.set_ylim([ymin,ymax])
        ax_ref.set_zlim([zmin,zmax])

        ax_initial.set_xlim([xmin,xmax])
        ax_initial.set_ylim([ymin,ymax])
        ax_initial.set_zlim([zmin,zmax])

        ax_both.view_init(30, angle)
        ax_curr.view_init(30, angle)
        ax_ref.view_init(30, angle)
        ax_initial.view_init(30, angle)


    ax_both.legend()
    ax_curr.legend()
    ax_ref.legend()
    ax_initial.legend()

    ax_both.grid(False)
    ax_both.set_xticks([])
    ax_both.set_yticks([])
    ax_both.set_zticks([])
    ax_curr.grid(False)
    ax_curr.set_xticks([])
    ax_curr.set_yticks([])
    ax_curr.set_zticks([])
    ax_ref.grid(False)
    ax_ref.set_xticks([])
    ax_ref.set_yticks([])
    ax_ref.set_zticks([])
    ax_initial.grid(False)
    ax_initial.set_xticks([])
    ax_initial.set_yticks([])
    ax_initial.set_zticks([])


    plt.savefig(title)
    plt.clf()
    plt.cla()
    plt.close(fig)

def get_render_mesh_from_sim(sim):
    cloth_verts = torch.stack([v.node.x for v in sim.cloths[0].mesh.verts]).float().to(device)
    cloth_faces = torch.Tensor([[vert.index for vert in f.v] for f in sim.cloths[0].mesh.faces]).to(device)

    pole_verts = torch.stack([v.node.x for v in sim.obstacles[0].curr_state_mesh.verts]).float().to(device)
    pole_faces = torch.Tensor([[vert.index for vert in f.v] for f in sim.obstacles[0].curr_state_mesh.faces]).to(device)
    pole_faces += len(cloth_verts)

    all_verts = [cloth_verts, pole_verts]
    all_faces = [cloth_faces, pole_faces]

    mesh = Meshes(verts=[torch.cat(all_verts)], faces=[torch.cat(all_faces)])
    return mesh

def get_cloth_mesh_from_sim(sim):
    cloth_verts = torch.stack([v.node.x for v in sim.cloths[0].mesh.verts]).float().to(device)
    cloth_faces = torch.Tensor([[vert.index for vert in f.v] for f in sim.cloths[0].mesh.faces]).to(device)
    all_verts = [cloth_verts]
    all_faces = [cloth_faces]
    mesh = Meshes(verts=[torch.cat(all_verts)], faces=[torch.cat(all_faces)])
    return mesh

def real2sim_transf(points, trans, scale, rot_inv):
    return ((points - trans)/scale)@rot_inv

def get_ref_pcl(sim_step, demo_dir):
    ref_pcl = np.load(os.path.join(demo_dir, '%03d.npy'%sim_step))
    ref_pcl = torch.from_numpy(ref_pcl).to(device).unsqueeze(0).float()
    return ref_pcl

def get_loss_per_iter(sim, epoch, sim_step, demo_dir, save, initial_states=None):
    curr_mesh = get_render_mesh_from_sim(sim)
    curr_mesh_cloth_only = get_cloth_mesh_from_sim(sim)
    curr_pcl = sample_points_from_meshes(curr_mesh, num_points)
    curr_pcl_cloth_only = sample_points_from_meshes(curr_mesh_cloth_only, num_points)
    ref_pcl = get_ref_pcl(sim_step, demo_dir)

    loss_chamfer, _ = chamfer_distance_one_sided(curr_pcl_cloth_only, ref_pcl)
    #loss_chamfer, _ = chamfer_distance_one_sided(ref_pcl, curr_pcl)
    #loss_chamfer, _ = chamfer_distance_one_sided(curr_pcl, ref_pcl)
    #loss_chamfer, _ = chamfer_distance(curr_pcl, ref_pcl)
    if (save):
        initial_mesh = initial_states[sim_step]
        initial_pcl = sample_points_from_meshes(initial_mesh, num_points)
        plot_pointclouds([curr_pcl, ref_pcl, initial_pcl], title='%s/fixed_epoch%02d-%03d'%(out_path,epoch,sim_step))
        plot_pointclouds([curr_pcl, ref_pcl, initial_pcl], title='%s/rot_epoch%02d-%03d'%(out_path,epoch,sim_step), angle=(sim_step/total_steps)*95)
    return loss_chamfer, curr_mesh

def run_sim(steps, sim, epoch, demo_dir, save, given_params=None, initial_states=None):
    if given_params is None:
        stretch_multiplier, mass_multiplier  = torch.sigmoid(param_g)
    else:
        stretch_multiplier, mass_multiplier  = given_params
    loss = 0.0

    orig_stretch = sim.cloths[0].materials[0].stretchingori

    new_stretch_multiplier = scale(stretch_multiplier, low_stretch, high_stretch)
    new_mass_mult = scale(mass_multiplier, low_mass, high_mass)

    sim.cloths[0].materials[0].stretchingori = orig_stretch*new_stretch_multiplier

    for node in sim.cloths[0].mesh.nodes:
        node.m  *= new_mass_mult
    arcsim.reload_material_data(sim.cloths[0].materials[0])

    print("mass, stretch", (new_mass_mult, new_stretch_multiplier))
    print(param_g.grad)

    mesh_states = []
    updates = 0
    for step in range(steps):
        print(step)
        arcsim.sim_step()
        loss_curr, curr_mesh = get_loss_per_iter(sim, epoch, step, demo_dir, save=save, initial_states=initial_states)
        #loss = loss_curr
        if step in [24]:
            loss += loss_curr
            updates += 1
        mesh_states.append(curr_mesh)

    loss /= updates

    return loss, mesh_states

def do_train(cur_step,optimizer,sim,initial_states):
    epoch = 0
    loss = float('inf')

    prev_loss = float('inf')
    final_param_g = None

    #thresh = 0.001
    thresh = 0.0005
    num_steps_to_run = total_steps
    losses = []
    params = []
    while True:
        
        reset_sim(sim, epoch)
        
        st = time.time()
        loss,_ = run_sim(num_steps_to_run, sim, epoch, demo_dir, save=False, initial_states=initial_states)
        #loss,_ = run_sim(num_steps_to_run, sim, epoch, demo_dir, save=(epoch%20==0), initial_states=initial_states)
        print('loss', loss)

        if loss < prev_loss:
            prev_loss = loss
            final_param_g = torch.sigmoid(param_g)

        losses.append(loss.item())
            
        if epoch > 50 or loss < thresh:
            break

        stretch_multiplier, mass_multiplier  = torch.sigmoid(param_g)
        new_stretch_multiplier = scale(stretch_multiplier, low_stretch, high_stretch)
        new_mass_multiplier = scale(mass_multiplier, low_mass, high_mass)
        params.append([new_stretch_multiplier, new_mass_multiplier])

        en0 = time.time()
        optimizer.zero_grad()
        step = True
        loss.backward()
        en1 = time.time()

        optimizer.step()
        print("=======================================")
        print('epoch {}: loss={}\n'.format(epoch, loss.data))
        
        print('forward tim = {}'.format(en0-st))
        print('backward time = {}'.format(en1-en0))
       
        epoch = epoch + 1
        # break

    return final_param_g, losses, params, epoch


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--demo_dir', type=str, default=os.path.join('band_sim_pcls/8'))
    parser.add_argument('-p', '--initial_params_file', type=str, default='')
    args = parser.parse_args()
    demo_dir = args.demo_dir
    demo_name = demo_dir.split('/')[-1]
    with open(out_path+('/log%s.txt'%timestamp),'w',buffering=1) as f:
        tot_step = 1
        sim=arcsim.get_sim()

        if args.initial_params_file: 
            meteornet_preds = np.load(args.initial_params_file, allow_pickle=True)
            initial_stretch, initial_mass = meteornet_preds.item()[demo_name]
            initial_stretch = scale(np.clip(initial_stretch, low_stretch+0.1, high_stretch-0.1), low_stretch, high_stretch, inverse=True)
            initial_mass = scale(np.clip(initial_mass, low_mass+0.1, high_mass-0.1), low_mass, high_mass, inverse=True)
            print(initial_stretch, initial_mass)
        else:
            initial_stretch = scale(np.mean((low_stretch, high_stretch)), low_stretch, high_stretch, inverse=True)
            initial_mass = scale(np.mean((low_mass, high_mass)), low_mass, high_mass, inverse=True)
        initial_probs = torch.tensor([initial_stretch,initial_mass])
    
        param_g = torch.log(initial_probs/(torch.ones_like(initial_probs)-initial_probs))
        print("here", torch.sigmoid(param_g), initial_probs)
        param_g.requires_grad = True
        lr = 0.4 # WORKS WELL pri
        optimizer = torch.optim.Adam([param_g],lr=lr)
        reset_sim(sim, 0)
        _, initial_states = run_sim(total_steps, sim, 0, demo_dir, save=False)
        final_param_g,losses,params,iters = do_train(tot_step,optimizer,sim,initial_states)
        reset_sim(sim, iters+1)
        _, _,  = run_sim(total_steps, sim, iters+1, demo_dir, save=True, given_params=final_param_g, initial_states=initial_states)
        final_stretch = scale(final_param_g[0], low_stretch, high_stretch)
        final_mass = scale(final_param_g[1], low_mass, high_mass)
        np.save('default_out_exp/params.npy', params)
        np.save('default_out_exp/losses.npy', losses)
        print('final_stretch', 'final_mass', final_stretch, final_mass)
        
    print("done")

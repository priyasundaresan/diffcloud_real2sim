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

def test_lift():
    if not os.path.exists('default_out'):
        os.mkdir('default_out')
    sim = arcsim.get_sim()
    positions = []
    arcsim.init_physics(os.path.join('conf/rigidcloth/lift/demo_diamond25.json'),'default_out/out0', False)
    stretch = sim.cloths[0].materials[0].stretchingori
    gravity = sim.gravity
    bend = sim.cloths[0].materials[0].bendingori
    drag = sim.wind.drag

    stretch_mult = 1
    mass_mult = 10
    drag_mult = 0
    
    sim.wind.drag = drag*drag_mult
    sim.cloths[0].materials[0].stretchingori = stretch*stretch_mult
    arcsim.reload_material_data(sim.cloths[0].materials[0])

    corner_idxs = [5,9,10,11,17,21,22,23,27,28,33,35,36,37,39,43,45,46,47,48]
    print('here',len(sim.cloths[0].mesh.nodes))
    for i,node in enumerate(sim.cloths[0].mesh.nodes):
        if i in corner_idxs:
            node.m  *= mass_mult

    for step in range(25):
        positions.append(sim.cloths[0].mesh.nodes[13].x.detach().cpu().numpy())
        print(step, np.round(positions[-1], 4))
        arcsim.sim_step()
    print('saving')
    np.save('lift_sim_traj.npy', np.array(positions))

def test_fold():
    if not os.path.exists('default_out'):
        os.mkdir('default_out')
    sim = arcsim.get_sim()
    positions = []
    arcsim.init_physics(os.path.join('conf/rigidcloth/lift/demo_fold.json'),'default_out/out0', False)
    stretch = sim.cloths[0].materials[0].stretchingori
    gravity = sim.gravity
    bend = sim.cloths[0].materials[0].bendingori
    drag = sim.wind.drag

    stretch_mult = 1
    mass_mult = 1
    drag_mult = 0

    #corner_idxs = [2,9,10,13,23,24,25,26,27,38,46,49,59,60,61,62,63,66]
    corner_idxs = [9,10,11,21,22,36,37,43,45,46,47,48]
    for i,node in enumerate(sim.cloths[0].mesh.nodes):
        if i in corner_idxs:
            node.m  *= mass_mult
    
    sim.wind.drag = drag*drag_mult
    sim.cloths[0].materials[0].stretchingori = stretch*stretch_mult
    arcsim.reload_material_data(sim.cloths[0].materials[0])

    for node in sim.cloths[0].mesh.nodes:
        node.m  *= mass_mult

    for step in range(25):
        positions.append(sim.cloths[0].mesh.nodes[42].x.detach().cpu().numpy())
        print(step, np.round(positions[-1], 4))
        arcsim.sim_step()
    print('saving')
    np.save('fold_sim_traj.npy', np.array(positions))

def test_hang():
    if not os.path.exists('default_out'):
        os.mkdir('default_out')
    sim = arcsim.get_sim()
    arcsim.init_physics(os.path.join('conf/rigidcloth/cloth_hang/demo2.json'),'default_out/out0', False)
    stretch = sim.cloths[0].materials[0].stretchingori

    stretch_mult = 1
    mass_mult = 10

    sim.cloths[0].materials[0].stretchingori = stretch*stretch_mult
    arcsim.reload_material_data(sim.cloths[0].materials[0])

    corner_idxs = [38,39,40,42,43,47,49,51,54,55,56,57,58,59,60,61,62,63,64,65,67,68]
    for i,node in enumerate(sim.cloths[0].mesh.nodes):
        if i in corner_idxs:
            node.m  *= mass_mult
    #for node in sim.cloths[0].mesh.nodes:
    #    node.m  *= mass_mult

    for step in range(15):
        print(step)
        arcsim.sim_step()

def test_band():
    if not os.path.exists('default_out'):
        os.mkdir('default_out')
    sim = arcsim.get_sim()
    arcsim.init_physics(os.path.join('conf/rigidcloth/band_stretch/demo.json'),'default_out/out0', False)
    stretch = sim.cloths[0].materials[0].stretchingori

    stretch_mult = 5
    mass_mult = 0.1

    sim.cloths[0].materials[0].stretchingori = stretch*stretch_mult
    arcsim.reload_material_data(sim.cloths[0].materials[0])

    for node in sim.cloths[0].mesh.nodes:
        node.m  *= mass_mult

    for step in range(25):
        print(step)
        arcsim.sim_step()

if __name__ == '__main__':
    #test_band()
    #test_hang()
    test_fold()
    #test_lift()

import os
import sys
import torch
import arcsim

def get_last_saved_folder():
    fnames = os.listdir('default_out')
    fnames = [f for f in fnames if f != 'out' and 'out' in f]
    fnames_sorted = sorted(fnames, key=lambda x: int(x[3:]))
    return os.path.join('default_out', fnames_sorted[-1])

with torch.autograd.profiler.profile() as prof:
    arcsim.msim(3,['arcsim','replay',get_last_saved_folder()])

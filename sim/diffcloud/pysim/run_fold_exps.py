#!/usr/bin/env python
import os
import sys

if __name__ == '__main__':
    pcls_dir = 'fold_real_pcls_gates'
    output_dir = 'fold_exps_diffcloud'
    if not os.path.exists(output_dir):
        os.mkdir(output_dir)
    for f in os.listdir(pcls_dir):
        if os.path.isdir(os.path.join(pcls_dir,f)):
            os.system('python exp_learn_fold_real2sim.py --demo_dir %s/%s'%(pcls_dir, f))
            os.system('mv default_out_exp %s/%s'%(output_dir,f))

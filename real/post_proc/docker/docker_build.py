#!/usr/bin/env python
import os

if __name__=="__main__":
    cmd = "nvidia-docker build -t kinova-postproc . "
    code = os.system(cmd)

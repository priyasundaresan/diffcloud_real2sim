#!/usr/bin/env python3
import os

if __name__=="__main__":
    #cmd = "nvidia-docker run \
    #     --gpus all \
    #     -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics \
    #     -v %s:/host \
    #     -it diffsim" % (os.path.abspath(os.path.join(os.getcwd(), '..')))
    cmd = "docker run \
         --gpus all \
         --runtime=nvidia \
         -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics \
         -v %s:/host \
         -it diffsim" % (os.path.abspath(os.path.join(os.getcwd(), '..')))
    print(cmd)
    code = os.system(cmd)

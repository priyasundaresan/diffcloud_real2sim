#!/usr/bin/env python
import os

if __name__=="__main__":
    cmd = "xhost +local:root && \
         nvidia-docker run \
         --gpus all \
         -e DISPLAY=$DISPLAY \
         -e QT_X11_NO_MITSHM=1 \
         -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics \
         -v /tmp/.X11-unix:/tmp/.X11-unix \
         -v %s:/host \
         -it diffsim" % (os.path.abspath(os.path.join(os.getcwd(), '..')))
    print(cmd)
    code = os.system(cmd)

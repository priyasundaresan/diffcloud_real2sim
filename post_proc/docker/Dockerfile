FROM mawall/ubuntu16.04_base

# Packages
#RUN apt-get update && apt-get install -y libpcl-dev=1.7.2-14build1
RUN apt-get update && apt-get install -y libpcl-dev=1.7.2-14ubuntu0.1
RUN pip install -U setuptools && \
    pip install         \
        python-pcl \
        matplotlib \
        numpy \
        scipy \
        scikit-learn \
        opencv-python \
	pyyaml 
WORKDIR /host
CMD ["/bin/bash"]

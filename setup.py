from setuptools import setup

setup(name='kinova_percep',
      version='0.1',
      description='Deformable object manipulation for repeated/periodic tasks',
      author='',
      author_email='',
      license='',
      packages=['kinova_percep'],
      python_requires='>=3.7',
      install_requires=['numpy==1.21', 'pybullet==3.1.7', 'gym==0.20.0',
                        'scipy', 'tqdm', 'attrdict', 'matplotlib', 'imageio',
                        'opencv-python', 'scikit-video', 'moviepy', 'pyffmpeg',
                        'pyyaml', 'rospkg', 'shapely',
                        'ghalton', 'KDEpy==1.0.3', 
                        'transforms3d', 'urdfpy', 'PySimpleGUI'  # for PyBulletRecorder
                        ],
)

"""
Task information for hardware.

@contactrika
"""

import numpy as np

TASK_INFO = {
    'fling': {
        'edge_size_px': 0,
        'max_unmasked_z': 0.175,
        'init_pos': np.array([0.7, 0, 0.175]),
        'init_ori_deg': np.array([90, 90, 90]),  # deg
    },
    'loop': {
        'edge_size_px': 0,
        'max_unmasked_z': 1.0,
        'init_pos': np.array([0.8, 0.1, 0.275]),
        'init_ori_deg': np.array([150, -30, 90]),  # deg
    },
    'winding': {
        'edge_size_px': 35,
        'max_unmasked_z': 0.09,
        'init_pos': np.array([0.44, 0, 0.12]),
        'init_ori_deg': np.array([150, 0, 90]),  # deg
    },
    'wiping': {
        'edge_size_px': 0,
        'max_unmasked_z': 0.07,
        'init_pos': np.array([0.54, -0.22, 0.067]),
        'init_ori_deg': np.array([105.99, 4.90, 87.47]),  # deg
    },
}

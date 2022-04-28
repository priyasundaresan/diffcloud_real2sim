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
    'lift': {
        'edge_size_px': 0,
        'max_unmasked_z': 0.175,
        'init_pos': np.array([0.7, 0, 0.175]),
        'init_ori_deg': np.array([90, 90, 90]),  # deg
    },
    'fold': {
        'edge_size_px': 0,
        'max_unmasked_z': 0.175,
        'init_pos': np.array([0.7, 0, 0.175]),
        'init_ori_deg': np.array([90, 90, 90]),  # deg
    },
    'stretch': {
        'edge_size_px': 0,
        'max_unmasked_z': 1.0,
        'init_pos': np.array([0.8, 0.1, 0.275]),
        'init_ori_deg': np.array([150, -30, 90]),  # deg
    },
}

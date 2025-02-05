import numpy as np

# Micro Camera (ENA-10005-AS)
camera_instrincs_1 = np.array(
    [
        251.2799, 
        250.8960, 
        184.5338, 
        216.2572, 
        -0.1345, 
        -0.0272, 
        0.0, 
        0.0]
)

# Milad's Camera (Video mode 640x360)
camera_instrincs_2 = np.array(
    [
        933.3358 / 2,
        935.7416 / 2,
        623.2117 / 2,
        394.1779 / 2,
        0.0261,
        -0.1228,
        0.0,
        0.0,
    ]
)

# Milad's Camera (Image mode 1280x720)
camera_instrincs_3 = np.array(
    [
        933.3358,
        935.7416,
        623.2117,
        394.1779,
        0.0261,
        -0.1228,
        0.0,
        0.0,
    ]
)
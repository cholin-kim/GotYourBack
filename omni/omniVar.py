import numpy as np

# Kinematics variables
L1 = 0.09+0.035   # head height (base ~ head center) (m)
L2 = 0.1340       # upper arm length (m)
L3 = 0.08+0.0525  # lower arm length (m)


def dhparam(joints):
    q1, q2, q3, q4, q5, q6 = np.array(joints).T
    return np.array([[0, 0, L1, -np.pi/2-q1],
                     [-np.pi/2, 0, 0, -q2],
                     [0, L2, 0, -q3],
                     [np.pi/2, 0, -L3, q4],
                     [-np.pi/2, 0, 0, -np.pi/2-q5],
                     [np.pi/2, 0, 0, q6]])
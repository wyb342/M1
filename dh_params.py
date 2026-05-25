import numpy as np

# 标准 DH 参数: [a, d, alpha, theta_offset]
# 对于关节 i，关节变量为 theta_i = q[i] + theta_offset（此处 offset 均为 0）
FR3_DH = [
    [0,     0.333,  0,             0],   # Joint 1
    [0,     0,     -np.pi/2,       0],   # Joint 2
    [0,     0.316,  np.pi/2,       0],   # Joint 3
    [0.0825,0,      np.pi/2,       0],   # Joint 4
    [-0.0825,0.384, -np.pi/2,      0],   # Joint 5
    [0,     0,      np.pi/2,       0],   # Joint 6
    [0.088, 0,      np.pi/2,       0]    # Joint 7
]

# 法兰盘相对于关节 7 的固定偏移 (同样用 DH 形式，theta=0)
FLANGE_DH = [0, 0.107, 0, 0]   # a, d, alpha, theta_offset
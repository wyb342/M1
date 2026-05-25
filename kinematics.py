import numpy as np
from dh_params import FR3_DH, FLANGE_DH

def dh_transform(a, d, alpha, theta):
    """标准 DH 齐次变换矩阵"""
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    return np.array([
        [ct, -st, 0, a],
        [st*ca, ct*ca, -sa, -d*sa],
        [st*sa, ct*sa, ca, d*ca],
        [0, 0, 0, 1]
    ])

def forward_kinematics(q):
    T = np.eye(4)
    for i in range(7):
        a, d, alpha, theta_offset = FR3_DH[i]
        theta = q[i] + theta_offset
        Ti = dh_transform(a, d, alpha, theta)
        T = T @ Ti
    # 不乘法兰盘，直接返回 fr3_link7 位姿
    return T


def jacobian(q):
    """
    计算空间几何雅可比矩阵 (6x7)
    末端速度 v = J @ dq, v = [线速度; 角速度] 在世界坐标系下
    末端为 fr3_link7（不带法兰盘偏移）
    """
    T = np.eye(4)
    origins = []  # 每个关节坐标系原点在基坐标系中的位置
    z_axes = []  # 每个关节 Z 轴在基坐标系中的方向

    for i in range(7):
        a, d, alpha, theta_offset = FR3_DH[i]
        theta = q[i] + theta_offset
        Ti = dh_transform(a, d, alpha, theta)
        T = T @ Ti
        origins.append(T[:3, 3].copy())
        z_axes.append(T[:3, 2].copy())

    # 末端位置（fr3_link7，即最后一个关节的末端，未加法兰盘偏移）
    p_ee = origins[-1]  # 因为 T 经过所有关节后就是 fr3_link7 的位姿

    J = np.zeros((6, 7))
    for i in range(7):
        z = z_axes[i]
        p = origins[i]
        J[:3, i] = np.cross(z, p_ee - p)  # 线速度部分
        J[3:, i] = z  # 角速度部分
    return J
def jacobian_singular_values(q):
    """返回雅可比矩阵的奇异值（降序排列）"""
    J = jacobian(q)
    _, S, _ = np.linalg.svd(J)
    return S

def condition_number(q):
    """雅可比矩阵的条件数（最大奇异值/最小奇异值）"""
    S = jacobian_singular_values(q)
    return S[0] / S[-1] if S[-1] > 1e-10 else np.inf

def euler_zyx_from_rotation(R):
    """
    从旋转矩阵提取 ZYX 欧拉角 (α, β, γ) 弧度
    旋转顺序: R = Rz(α) * Ry(β) * Rx(γ)
    其中 α=yaw (绕Z), β=pitch (绕Y), γ=roll (绕X)
    返回 [α, β, γ] 即 [yaw, pitch, roll]
    """
    singular = np.abs(R[0,2]) > 0.999999
    if not singular:
        yaw = np.arctan2(R[1,0], R[0,0])
        pitch = np.arctan2(-R[2,0], np.sqrt(R[2,1]**2 + R[2,2]**2))
        roll = np.arctan2(R[2,1], R[2,2])
    else:
        # 万向锁情况 (pitch = ±π/2)
        yaw = 0.0
        pitch = -np.sign(R[0,2]) * np.pi / 2
        roll = np.arctan2(R[0,1], R[0,0])
    return np.array([yaw, pitch, roll])

def jacobian_analytic(q):
    """
    解析雅可比 (6x7): [线速度; 欧拉角速率 (ZYX)] = J_analytic @ dq
    
    对于 ZYX 欧拉角 (yaw-pitch-roll)，角速度与欧拉角速率的关系为:
    ω = B * [yaw_dot; pitch_dot; roll_dot]
    
    B 矩阵为:
        [0, -sin(yaw), cos(yaw)*cos(pitch)]
    B = [0,  cos(yaw), sin(yaw)*cos(pitch)]
        [1,  0,       -sin(pitch)       ]
    """
    # 几何雅可比
    J_geom = jacobian(q)          # (6,7)
    J_v = J_geom[:3, :]           # 线速度部分直接保留
    J_omega = J_geom[3:, :]       # 角速度部分 (世界坐标系下的角速度)

    # 当前末端姿态的欧拉角 (ZYX: yaw, pitch, roll)
    T_ee = forward_kinematics(q)
    R = T_ee[:3, :3]
    yaw, pitch, roll = euler_zyx_from_rotation(R)

    # 构造 B 矩阵: 角速度 = B * [yaw_dot; pitch_dot; roll_dot]
    sy, cy = np.sin(yaw), np.cos(yaw)
    sp, cp = np.sin(pitch), np.cos(pitch)
    
    B = np.array([
        [0, -sy, cy*cp],
        [0, cy, sy*cp],
        [1, 0, -sp]
    ])

    # 避免万向锁，若 cp 太小则用伪逆，否则直接求逆
    if abs(cp) < 1e-6:
        # 接近奇异，使用伪逆
        B_inv = np.linalg.pinv(B)
    else:
        B_inv = np.linalg.inv(B)

    J_euler = B_inv @ J_omega   # (3,7)

    # 拼接
    J_analytic = np.vstack([J_v, J_euler])
    return J_analyti
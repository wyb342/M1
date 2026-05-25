import numpy as np
import matplotlib.pyplot as plt
from kinematics import jacobian

# 定义每个关节的运动范围（弧度）（基于 FR3 官方限位）
joint_limits = [
    (-2.8973, 2.8973),  # J1
    (-1.7628, 1.7628),  # J2
    (-2.8973, 2.8973),  # J3
    (-3.0718, -0.0698), # J4（注意：范围是负值）
    (-2.8973, 2.8973),  # J5
    (-0.0175, 3.7525),  # J6
    (-2.8973, 2.8973)   # J7
]

# 采样点数
n_points = 200

# 存储每个关节的最小奇异值曲线
min_sv_curves = []
joint_angles = []

for i in range(7):
    low, high = joint_limits[i]
    angles = np.linspace(low, high, n_points)
    min_sv = []
    for ang in angles:
        q = np.zeros(7)
        q[i] = ang
        J = jacobian(q)
        sv = np.linalg.svd(J, compute_uv=False)
        min_sv.append(np.min(sv))
    min_sv_curves.append(min_sv)
    joint_angles.append(angles)

# 方式1：所有关节画在同一张图上（不同颜色）
plt.figure(figsize=(12, 6))
for i in range(7):
    plt.plot(joint_angles[i], min_sv_curves[i], label=f'Joint {i+1}')
plt.xlabel('Joint angle (rad)')
plt.ylabel('Minimum singular value')
plt.title('Minimum singular value vs joint angle (other joints at 0)')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# 方式2：画7个子图，方便观察每个关节
fig, axes = plt.subplots(2, 4, figsize=(14, 8))  # 2x4，最后一个子图隐藏
axes = axes.flatten()
for i in range(7):
    ax = axes[i]
    ax.plot(joint_angles[i], min_sv_curves[i], linewidth=2)
    ax.set_title(f'Joint {i+1}')
    ax.set_xlabel('Angle (rad)')
    ax.set_ylabel('Min singular value')
    ax.grid(True)
    # 标记奇异值接近0的区域（如果低于0.01）
    threshold = 0.01
    for idx, val in enumerate(min_sv_curves[i]):
        if val < threshold:
            ax.axvline(x=joint_angles[i][idx], color='red', linestyle='--', alpha=0.3)
# 隐藏第8个子图
axes[7].set_visible(False)
plt.tight_layout()
plt.show()

# 打印奇异值过低的角度（<0.01 认为接近奇异）
print("各关节导致奇异值低于0.01的角度范围：")
for i in range(7):
    angles = joint_angles[i]
    svs = min_sv_curves[i]
    low_indices = np.where(np.array(svs) < 0.01)[0]
    if len(low_indices) > 0:
        angle_range = (angles[low_indices[0]], angles[low_indices[-1]])
        print(f"Joint {i+1}:  {angle_range[0]:.2f} ~ {angle_range[1]:.2f} rad")
    else:
        print(f"Joint {i+1}: 无奇异风险（在整个运动范围内最小奇异值 >0.01）")
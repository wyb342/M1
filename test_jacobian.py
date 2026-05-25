import numpy as np
from kinematics import forward_kinematics, jacobian

def finite_diff_jacobian(q, eps=1e-8):
    """通过有限差分近似计算雅可比，用于验证"""
    J_num = np.zeros((6, 7))
    T0 = forward_kinematics(q)
    p0 = T0[:3, 3]
    R0 = T0[:3, :3]
    for i in range(7):
        dq = np.zeros(7)
        dq[i] = eps
        q_plus = q + dq
        T1 = forward_kinematics(q_plus)
        p1 = T1[:3, 3]
        R1 = T1[:3, :3]
        # 线速度部分
        J_num[:3, i] = (p1 - p0) / eps
        # 角速度部分: 使用旋转矩阵差分的反对称矩阵提取
        dR = (R1 @ R0.T - np.eye(3)) / eps
        omega = np.array([dR[2,1], dR[0,2], dR[1,0]])  # 近似角速度
        J_num[3:, i] = omega
    return J_num

# 随机测试多组 q
np.random.seed(42)
n_tests = 10
max_err = 0.0
errors = []

print("开始验证几何雅可比 (有限差分对比)...")
for i in range(n_tests):
    q_test = np.random.uniform(-1, 1, 7)  # 简化，实际可考虑关节限位
    J_ana = jacobian(q_test)
    J_num = finite_diff_jacobian(q_test)
    err = np.linalg.norm(J_ana - J_num, ord='fro')
    errors.append(err)
    max_err = max(max_err, err)
    print(f"测试 {i+1}: 误差 (Frobenius 范数): {err:.2e}")

mean_err = np.mean(errors)
print(f"\n验证结果统计:")
print(f"  最大误差: {max_err:.2e}")
print(f"  平均误差: {mean_err:.2e}")

if max_err < 1e-5:
    print("✅ 雅可比验证通过！")
else:
    print("⚠️ 误差较大，请检查雅可比计算是否正确。")

# 输出并保存一个示例雅可比矩阵（q=0）
q0 = np.zeros(7)
J0 = jacobian(q0)
print(f"\n示例: q = zeros(7) 时的几何雅可比矩阵 (6x7):")
print(np.array2string(J0, formatter={'float_kind': lambda x: f"{x:8.4f}"}))

# 保存到 CSV 文件
np.savetxt("geometric_jacobian_example.csv", J0, delimiter=",", fmt="%.6f")
print("\n已保存到 geometric_jacobian_example.csv")

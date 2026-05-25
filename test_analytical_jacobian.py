#!/usr/bin/env python3
"""
test_analytical_jacobian.py
验证解析雅可比 (J_analytic) 与数值差分结果的一致性。
数值差分直接计算 (位置差分→线速度，欧拉角差分→欧拉角速率)
"""

import numpy as np
from kinematics import forward_kinematics, jacobian_analytic, euler_zyx_from_rotation

def finite_diff_jacobian_analytic(q, eps=1e-8):
    """
    用有限差分逼近解析雅可比矩阵。
    返回 6x7 矩阵: [线速度; 欧拉角速率] 对关节角的偏导数。
    """
    n = len(q)
    J_num = np.zeros((6, n))
    # 当前状态
    T0 = forward_kinematics(q)
    pos0 = T0[:3, 3]
    euler0 = euler_zyx_from_rotation(T0[:3, :3])
    for i in range(n):
        dq = np.zeros(n)
        dq[i] = eps
        q_plus = q + dq
        T1 = forward_kinematics(q_plus)
        pos1 = T1[:3, 3]
        euler1 = euler_zyx_from_rotation(T1[:3, :3])
        # 线速度部分
        J_num[:3, i] = (pos1 - pos0) / eps
        # 欧拉角部分：直接作差（注意角度跳变处理）
        deuler = euler1 - euler0
        # 规范化到 [-π, π) 范围内，避免因角度周期导致的跳跃
        deuler = (deuler + np.pi) % (2*np.pi) - np.pi
        J_num[3:, i] = deuler / eps
    return J_num

def main():
    # 关节限位（弧度），避免进入万向锁附近导致欧拉角差值跳变
    joint_ranges = [
        (-2.5, 2.5),   # J1
        (-1.5, 1.5),   # J2
        (-2.5, 2.5),   # J3
        (-2.8, -0.2),  # J4 (注意实际为负)
        (-2.5, 2.5),   # J5
        (-0.01, 3.5),  # J6
        (-2.5, 2.5)    # J7
    ]

    np.random.seed(42)
    n_tests = 30
    errors = []
    print("开始验证解析雅可比 (有限差分对比)...")
    for _ in range(n_tests):
        q = np.array([np.random.uniform(low, high) for low, high in joint_ranges])
        J_ana = jacobian_analytic(q)
        J_num = finite_diff_jacobian_analytic(q)
        err = np.linalg.norm(J_ana - J_num, ord='fro')
        errors.append(err)

    max_err = np.max(errors)
    mean_err = np.mean(errors)
    print(f"Frobenius 误差: 最大 = {max_err:.2e}, 平均 = {mean_err:.2e}")

    if max_err < 1e-5:
        print("✅ 解析雅可比验证通过！")
    else:
        print("⚠️ 误差较大，可能原因：欧拉角提取函数有误、万向锁、或差分步长不当。")

    # 额外打印一个 q=0 时的解析雅可比作为示例
    q0 = np.zeros(7)
    J0 = jacobian_analytic(q0)
    print("\n示例: q = zeros(7) 时的解析雅可比矩阵 (6x7):")
    print(np.array2string(J0, formatter={'float_kind': lambda x: f"{x:8.4f}"}))
    # 可选保存到文件
    np.savetxt("analytic_jacobian_example.csv", J0, delimiter=",", fmt="%.6f")
    print("已保存到 analytic_jacobian_example.csv")

if __name__ == "__main__":
    main()
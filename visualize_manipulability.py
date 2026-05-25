#!/usr/bin/env python3
"""
visualize_manipulability.py
可视化 Franka FR3 机器人的可操作度椭球（Manipulability Ellipsoid）
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from kinematics import forward_kinematics, jacobian, jacobian_singular_values


def manipulability_measure(q):
    """
    计算 Yoshikawa 可操作度指标
    w = sqrt(det(J * J^T))
    """
    J = jacobian(q)
    Jv = J[:3, :]  # 线速度部分的雅可比
    M = Jv @ Jv.T
    return np.sqrt(np.linalg.det(M))


def manipulability_ellipsoid_data(q, scale=1.0):
    """
    计算可操作度椭球的数据
    
    参数:
        q: 关节角度配置 (7,)
        scale: 椭球缩放因子，用于可视化
        
    返回:
        center: 椭球中心（末端执行器位置）
        axes: 椭球的三个主轴方向
        radii: 三个主轴的长度（奇异值）
        x, y, z: 用于绘图的椭球表面点
    """
    J = jacobian(q)
    Jv = J[:3, :]  # 只考虑线速度部分 (3x7)
    
    # 计算 JJ^T
    M = Jv @ Jv.T
    
    # 特征值分解
    eigenvalues, eigenvectors = np.linalg.eigh(M)
    
    # 按降序排列
    idx = np.argsort(eigenvalues)[::-1]
    eigenvalues = eigenvalues[idx]
    eigenvectors = eigenvectors[:, idx]
    
    # 奇异值是特征值的平方根
    singular_values = np.sqrt(np.maximum(eigenvalues, 0))
    
    # 椭球中心（末端位置）
    T_ee = forward_kinematics(q)
    center = T_ee[:3, 3]
    
    # 生成椭球表面点
    u = np.linspace(0, 2 * np.pi, 30)
    v = np.linspace(0, np.pi, 20)
    
    # 单位球面
    x_unit = np.outer(np.cos(u), np.sin(v))
    y_unit = np.outer(np.sin(u), np.sin(v))
    z_unit = np.outer(np.ones(np.size(u)), np.cos(v))
    
    # 将单位球面转换为椭球
    # 椭球方程: r = E * sphere，其中 E 的列是主轴方向乘以半径
    radii = singular_values * scale
    
    # 变换到椭球
    points = np.zeros((3, len(u), len(v)))
    for i in range(len(u)):
        for j in range(len(v)):
            sphere_point = np.array([x_unit[i, j], y_unit[i, j], z_unit[i, j]])
            ellipsoid_point = eigenvectors @ (radii * sphere_point)
            points[:, i, j] = ellipsoid_point + center
    
    x = points[0, :, :]
    y = points[1, :, :]
    z = points[2, :, :]
    
    return {
        'center': center,
        'axes': eigenvectors,
        'radii': singular_values,
        'x': x,
        'y': y,
        'z': z,
        'manipulability': np.prod(singular_values)
    }


def plot_manipulability_ellipsoid(q, scale=0.3, save_path=None):
    """
    绘制可操作度椭球
    
    参数:
        q: 关节角度配置 (7,)
        scale: 椭球可视化缩放因子
        save_path: 保存图像的路径（可选）
    """
    # 计算椭球数据
    ellipsoid = manipulability_ellipsoid_data(q, scale)
    
    # 创建图形
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # 绘制椭球表面
    ax.plot_surface(ellipsoid['x'], ellipsoid['y'], ellipsoid['z'], 
                    alpha=0.3, color='cyan', edgecolor='none')
    
    # 绘制椭球的三个主轴
    center = ellipsoid['center']
    axes = ellipsoid['axes']
    radii = ellipsoid['radii'] * scale
    
    colors = ['red', 'green', 'blue']
    labels = ['Max', 'Mid', 'Min']
    
    for i in range(3):
        axis_dir = axes[:, i]
        # 绘制双向箭头表示主轴
        ax.plot([center[0] - radii[i]*axis_dir[0], center[0] + radii[i]*axis_dir[0]],
                [center[1] - radii[i]*axis_dir[1], center[1] + radii[i]*axis_dir[1]],
                [center[2] - radii[i]*axis_dir[2], center[2] + radii[i]*axis_dir[2]],
                color=colors[i], linewidth=2, label=f'{labels[i]} axis')
    
    # 标记椭球中心
    ax.scatter(*center, color='black', s=100, marker='o', label='End-effector')
    
    # 设置标签和标题
    ax.set_xlabel('X (m)', fontsize=10)
    ax.set_ylabel('Y (m)', fontsize=10)
    ax.set_zlabel('Z (m)', fontsize=10)
    
    manip_measure = ellipsoid['manipulability']
    singular_vals = ellipsoid['radii']
    
    title = f'Manipulability Ellipsoid\n'
    title += f'Yoshikawa Index: {manip_measure:.4f}\n'
    title += f'Singular Values: [{singular_vals[0]:.3f}, {singular_vals[1]:.3f}, {singular_vals[2]:.3f}]'
    
    ax.set_title(title, fontsize=11)
    ax.legend(loc='upper right', fontsize=9)
    
    # 设置等比例坐标轴
    max_range = np.max(radii) * 1.5
    ax.set_xlim(center[0] - max_range, center[0] + max_range)
    ax.set_ylim(center[1] - max_range, center[1] + max_range)
    ax.set_zlim(center[2] - max_range, center[2] + max_range)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"图像已保存到: {save_path}")
    
    plt.show()


def compare_multiple_configurations(configurations, joint_names=None, save_path=None):
    """
    比较多个关节配置的可操作度
    
    参数:
        configurations: 字典，键为配置名称，值为关节角度数组
        joint_names: 关节名称列表（可选）
        save_path: 保存 CSV 结果的路径（可选）
    """
    results = []
    
    print("=" * 80)
    print("可操作度分析结果")
    print("=" * 80)
    print(f"{'配置名称':<20} {'可操作度':<15} {'条件数':<15} {'最小奇异值':<15}")
    print("-" * 80)
    
    for name, q in configurations.items():
        J = jacobian(q)
        Jv = J[:3, :]
        singular_values = jacobian_singular_values(q)[:3]  # 只取前3个（线速度部分）
        
        # 计算可操作度指标
        manip = np.prod(singular_values)
        
        # 计算条件数
        condition = singular_values[0] / singular_values[-1] if singular_values[-1] > 1e-10 else np.inf
        
        results.append({
            'name': name,
            'manipulability': manip,
            'condition_number': condition,
            'singular_values': singular_values,
            'q': q
        })
        
        print(f"{name:<20} {manip:<15.4f} {condition:<15.2f} {singular_values[-1]:<15.6f}")
    
    print("=" * 80)
    
    # 保存结果到 CSV
    if save_path:
        with open(save_path, 'w') as f:
            f.write("Configuration,Manipulability,Condition Number,Sigma1,Sigma2,Sigma3\n")
            for r in results:
                s = r['singular_values']
                f.write(f"{r['name']},{r['manipulability']:.6f},{r['condition_number']:.6f},")
                f.write(f"{s[0]:.6f},{s[1]:.6f},{s[2]:.6f}\n")
        print(f"\n结果已保存到: {save_path}")
    
    return results


def main():
    """主函数：演示可操作度椭球可视化"""
    
    # 定义几个典型的关节配置
    configurations = {
        'Zero Configuration': np.zeros(7),
        'Extended Arm': np.array([0, -np.pi/4, 0, -3*np.pi/4, 0, np.pi/2, np.pi/4]),
        'Folded Arm': np.array([np.pi/4, np.pi/3, np.pi/4, -np.pi/2, -np.pi/4, np.pi/6, 0]),
        'Random Config 1': np.array([0.5, -0.5, 0.3, -1.5, 0.2, 1.0, 0.5]),
        'Random Config 2': np.array([-0.3, 0.7, -0.5, -2.0, 0.8, 0.5, -0.3]),
    }
    
    # 比较多个配置的可操作度
    results = compare_multiple_configurations(
        configurations, 
        save_path='manipulability_comparison.csv'
    )
    
    # 可视化其中一个配置的可操作度椭球
    print("\n正在生成可操作度椭球可视化...")
    
    # 选择 Extended Arm 配置进行可视化
    q_viz = configurations['Extended Arm']
    plot_manipulability_ellipsoid(
        q_viz, 
        scale=0.3,
        save_path='manipulability_ellipsoid.png'
    )
    
    print("\n提示:")
    print("- 可操作度指标越大，机器人运动能力越好")
    print("- 条件数越接近 1，各向同性越好")
    print("- 最小奇异值接近 0 表示接近奇异位形")


if __name__ == "__main__":
    main()

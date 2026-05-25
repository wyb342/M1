
import numpy as np
from dh_params import FR3_DH
from kinematics import forward_kinematics, jacobian

class GravityTerm:
    """
    机器人重力项计算器
    
    重力项 g(q) 在机器人动力学方程中出现：
    τ = M(q)q̈ + C(q,q̇)q̇ + g(q)
    
    其中 g(q) 是由于重力产生的关节力矩向量
    """
    
    def __init__(self):
        # Franka Emika FR3 各连杆的质量 (kg)
        self.masses = np.array([
            2.9,   # link 1
            2.7,   # link 2  
            2.4,   # link 3
            2.3,   # link 4
            1.8,   # link 5
            1.5,   # link 6
            0.5    # link 7 (末端执行器较轻)
        ])
        
        # 重力加速度向量 (世界坐标系)
        self.g_vector = np.array([0, 0, -9.81])  # m/s^2
        
        # 各连杆质心位置（相对于各自坐标系原点）
        # 这里使用简化的估计值，实际应用中应从CAD模型或参数辨识获得
        self.centroids = [
            np.array([0, 0, 0.15]),   # link 1 质心
            np.array([0, -0.1, 0]),   # link 2 质心
            np.array([0, 0, 0.15]),   # link 3 质心
            np.array([0, 0.1, 0]),    # link 4 质心
            np.array([0, 0, 0.15]),   # link 5 质心
            np.array([0, 0, 0.1]),    # link 6 质心
            np.array([0, 0, 0.05])    # link 7 质心
        ]
    
    def compute_gravity_term(self, q):
        """
        计算重力项 g(q)
        
        方法：基于雅可比矩阵和势能推导
        g_i = Σ(m_j * g^T * J_vi_j) 对于所有 j >= i
        
        其中：
        - m_j 是连杆 j 的质量
        - g 是重力加速度向量
        - J_vi_j 是连杆 j 质心的线速度雅可比矩阵的第 i 列
        
        参数:
        q: 关节角度向量 (7,)
        
        返回:
        g_q: 重力项向量 (7,)
        """
        n_links = len(q)
        g_q = np.zeros(n_links)
        
        # 计算每个连杆对重力项的贡献
        for j in range(n_links):
            # 计算连杆 j 质心的位置和雅可比
            centroid_jacobian = self._compute_centroid_jacobian(q, j)
            
            # 连杆 j 对所有相关关节的重力贡献
            for i in range(j + 1):
                # 重力在该关节上的投影
                gravity_projection = np.dot(self.g_vector, centroid_jacobian[:3, i])
                g_q[i] += self.masses[j] * gravity_projection
        
        return g_q
    
    def _compute_centroid_jacobian(self, q, link_idx):
        """
        计算指定连杆质心的雅可比矩阵
        
        参数:
        q: 关节角度向量
        link_idx: 连杆索引
        
        返回:
        J_centroid: 质心的 6x(7) 雅可比矩阵
        """
        T = np.eye(4)
        origins = []
        z_axes = []
        
        # 正向运动学计算到指定连杆
        for i in range(link_idx + 1):
            a, d, alpha, theta_offset = FR3_DH[i]
            theta = q[i] + theta_offset
            Ti = self._dh_transform(a, d, alpha, theta)
            T = T @ Ti
            origins.append(T[:3, 3].copy())
            z_axes.append(T[:3, 2].copy())
        
        # 计算连杆质心在世界坐标系中的位置
        R_link = T[:3, :3]
        centroid_local = self.centroids[link_idx]
        centroid_world = R_link @ centroid_local + origins[-1]
        
        # 计算质心雅可比
        J_centroid = np.zeros((6, len(q)))
        for i in range(link_idx + 1):
            z = z_axes[i]
            p = origins[i]
            
            # 线速度部分
            J_centroid[:3, i] = np.cross(z, centroid_world - p)
            # 角速度部分
            J_centroid[3:, i] = z
        
        return J_centroid
    
    def _dh_transform(self, a, d, alpha, theta):
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
    
    def compute_gravity_potential_energy(self, q):
        """
        计算系统的重力势能
        
        V = Σ(m_i * g^T * p_ci)
        
        其中 p_ci 是第 i 个连杆质心的位置向量
        
        参数:
        q: 关节角度向量
        
        返回:
        V: 总重力势能
        """
        V = 0.0
        T = np.eye(4)
        
        for i in range(len(q)):
            a, d, alpha, theta_offset = FR3_DH[i]
            theta = q[i] + theta_offset
            Ti = self._dh_transform(a, d, alpha, theta)
            T = T @ Ti
            
            # 计算连杆质心位置
            R_link = T[:3, :3]
            p_origin = T[:3, 3]
            centroid_local = self.centroids[i]
            centroid_world = R_link @ centroid_local + p_origin
            
            # 累加势能
            V += self.masses[i] * np.dot(self.g_vector, centroid_world)
        
        return V
    
    def compute_gravity_numerical_gradient(self, q, eps=1e-6):
        """
        通过数值梯度方法计算重力项（用于验证）
        
        g(q) = ∂V/∂q ≈ [V(q+eps) - V(q-eps)] / (2*eps)
        
        参数:
        q: 关节角度向量
        eps: 差分步长
        
        返回:
        g_numeric: 数值计算的重力项
        """
        n = len(q)
        g_numeric = np.zeros(n)
        
        for i in range(n):
            q_plus = q.copy()
            q_minus = q.copy()
            
            q_plus[i] += eps
            q_minus[i] -= eps
            
            V_plus = self.compute_gravity_potential_energy(q_plus)
            V_minus = self.compute_gravity_potential_energy(q_minus)
            
            g_numeric[i] = (V_plus - V_minus) / (2 * eps)
        
        return g_numeric


def example_usage():
    """使用示例"""
    print("Franka FR3 重力项计算示例")
    print("=" * 50)
    
    # 创建重力计算器
    gravity_calc = GravityTerm()
    
    # 测试不同的关节构型
    test_configs = [
        ("零位", np.zeros(7)),
        ("伸展位形", np.array([0, -np.pi/4, 0, -3*np.pi/4, 0, np.pi/2, np.pi/4])),
        ("折叠位形", np.array([np.pi/4, np.pi/4, np.pi/4, np.pi/4, np.pi/4, np.pi/4, np.pi/4]))
    ]
    
    for name, q in test_configs:
        print(f"\n{name}: q = {np.round(q, 3)}")
        
        # 计算重力项
        g_q = gravity_calc.compute_gravity_term(q)
        print(f"重力项 g(q) = {np.round(g_q, 4)} N·m")
        
        # 计算势能
        V = gravity_calc.compute_gravity_potential_energy(q)
        print(f"重力势能 V = {np.round(V, 4)} J")
        
        # 数值验证
        g_numeric = gravity_calc.compute_gravity_numerical_gradient(q)
        print(f"数值梯度 g_num = {np.round(g_numeric, 4)} N·m")
        
        # 误差分析
        error = np.linalg.norm(g_q - g_numeric)
        print(f"解析解与数值解误差: {error:.2e}")


if __name__ == "__main__":
    example_usage()

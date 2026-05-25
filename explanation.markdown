# Franka FR3 机械臂运动学与动力学分析框架

## 📋 项目简介

本项目实现了Franka Emika FR3七自由度机械臂的完整运动学和动力学分析框架，包括正向运动学、雅可比矩阵计算、奇异性分析、可操作性分析和重力项推导等功能模块。

---

## 🗂️ 项目结构
---
- `M1/`
  - `dh_params.py` # DH参数定义
  - `kinematics.py` # 核心运动学函数
  - `gravity_term.py` # 重力项计算（新增）
  - `test_fk.py` # 正向运动学测试
  - `test_jacobian.py` # 几何雅可比测试
  - `test_analytical_jacobian.py` # 解析雅可比测试
  - `plot_singularity.py` # 奇异性可视化
  - `visualize_manipulability.py` # 可操作性椭球可视化
  - `franka_fr3/` # FR3机器人模型文件
    - `assets/` # 3D模型文件(.stl, .obj)
    - `fr3.xml` # MuJoCo模型描述
    - `scene.xml` # 场景配置
  - `analytic_jacobian_example.csv` # 解析雅可比示例数据
  - `geometric_jacobian_example.csv` # 几何雅可比示例数据
  - `manipulability_comparison.csv` # 可操作性对比数据
  - `manipulability_ellipsoid.png` # 可操作性椭球图

## 🔧 核心模块说明

### 1. DH参数定义 (`dh_params.py`)

定义了FR3机器人的标准DH参数：
**关键参数：**
- `a`: 连杆长度
- `d`: 连杆偏距
- `alpha`: 连杆扭转角
- `theta_offset`: 关节角度偏移

### 2. 运动学核心 (`kinematics.py`)

提供以下核心函数：

#### `forward_kinematics(q)`
- **功能**: 计算正向运动学
- **输入**: 关节角度向量 q (7,)
- **输出**: 末端执行器位姿矩阵 T (4×4)
- **说明**: 返回fr3_link7的位姿（不含法兰盘）

#### `jacobian(q)`
- **功能**: 计算空间几何雅可比矩阵
- **输入**: 关节角度向量 q (7,)
- **输出**: 雅可比矩阵 J (6×7)
- **说明**: v = J @ dq，其中v = [线速度; 角速度]

#### `jacobian_analytic(q)`
- **功能**: 计算解析雅可比矩阵
- **输入**: 关节角度向量 q (7,)
- **输出**: 解析雅可比矩阵 J_analytic (6×7)
- **说明**: 使用ZYX欧拉角表示姿态

#### `euler_zyx_from_rotation(R)`
- **功能**: 从旋转矩阵提取ZYX欧拉角
- **输入**: 旋转矩阵 R (3×3)
- **输出**: [yaw, pitch, roll] 弧度值

#### `condition_number(q)`
- **功能**: 计算雅可比矩阵条件数
- **输入**: 关节角度向量 q (7,)
- **输出**: 条件数标量
- **说明**: 用于奇异性判断

### 3. 重力项计算 (`gravity_term.py`) ⭐新增

#### `GravityTerm` 类

主要方法：

##### `compute_gravity_term(q)`
- **功能**: 计算重力项 g(q)
- **输入**: 关节角度向量 q (7,)
- **输出**: 重力力矩向量 g_q (7,)
- **原理**: 基于雅可比矩阵和势能梯度

##### `compute_gravity_potential_energy(q)`
- **功能**: 计算系统重力势能
- **输入**: 关节角度向量 q (7,)
- **输出**: 势能标量 V

##### `compute_gravity_numerical_gradient(q, eps=1e-6)`
- **功能**: 数值梯度验证
- **输入**: 关节角度向量 q，差分步长 eps
- **输出**: 数值计算的重力项
- **用途**: 验证解析解正确性

**物理模型：**
τ = M(q)q̈ + C(q,q̇)q̇ + g(q)

其中 g(q) 为重力产生的关节力矩。

---

## 🚀 快速开始

### 环境要求
bash Python >= 3.7 
依赖库: numpy, matplotlib
### 安装依赖
bash 
pip install numpy matplotlib
### 基本功能

#### 1. 正向运动学

#### 2. 雅可比矩阵计算
#### 3. 重力项计算
#### 4. 奇异性分析


## 📊 运行测试与可视化

### 运行所有测试
- **正向运动学测试**: `test_fk.py`
- **雅可比矩阵测试**: 
  - `test_jacobian.py` (几何雅可比)
  - `test_analytical_jacobian.py` (解析雅可比)
- **奇异性分析**: `plot_singularity.py`
- **重力项测试（包含数值验证）**: `gravity_term.py`
### 生成可视化图表
---
#### 奇异性可视化

python plot_singularity.py

#### 可操作性椭球

python visualize_manipulability.py
## 📐 理论背景

### DH参数约定

采用**标准DH参数**约定：
1. 绕 Z_i 旋转 θ_i
2. 沿 Z_i 平移 d_i
3. 沿 X_i 平移 a_i
4. 绕 X_i 旋转 α_i

### 雅可比矩阵

**几何雅可比**: 
- 线速度部分: J_v,i = z_i × (p_ee - p_i)
- 角速度部分: J_ω,i = z_i

**解析雅可比**:
- 通过欧拉角映射矩阵 B 转换
- J_analytic = [J_v; B⁻¹ @ J_ω]

### 重力项推导

**方法一：势能梯度法**

$$
g(q) = \frac{\partial V}{\partial q}
$$

$$
V = \sum m_i \cdot g^T \cdot p_{ci}
$$

**方法二：雅可比传递法**

$$
g_i = \sum_{j=i}^{n} m_j \mathbf{g}^T \mathbf{J}_{v,i,j}
$$

其中：
- $g_i$ 为第 $i$ 个关节的重力力矩分量
- $m_j$ 为第 $j$ 个连杆的质量
- $\mathbf{g}$ 为重力加速度向量
- $\mathbf{J}_{v,i,j}$ 为第 $j$ 个连杆质心相对于第 $i$ 个关节的线速度雅可比矩阵部分
- $n$ 为机械臂总自由度数（此处为7）



## 🔍 常见问题

### Q1: 如何修改机器人参数？

编辑 `dh_params.py` 中的 `FR3_DH` 列表，或修改 `gravity_term.py` 中的质量和质心参数。

### Q2: 数值验证误差较大怎么办？

检查：
1. 差分步长 eps 是否合适（推荐 1e-6）
2. 质心位置参数是否准确
3. 质量参数是否符合实际

### Q3: 如何处理奇异位形？

当条件数 > 100 时：
1. 避免在该位形附近操作
2. 使用阻尼最小二乘法
3. 切换到冗余自由度优化

### Q4: 如何扩展到其他机器人？

1. 在 `dh_params.py` 中定义新的DH参数
2. 调整 `gravity_term.py` 中的质量和质心
3. 保持接口一致性即可复用所有函数

---

## 📝 输出文件说明

| 文件名 | 内容 | 用途 |
|--------|------|------|
| `analytic_jacobian_example.csv` | 解析雅可比矩阵数据 | 对比分析 |
| `geometric_jacobian_example.csv` | 几何雅可比矩阵数据 | 对比分析 |
| `manipulability_comparison.csv` | 可操作性指标 | 性能评估 |
| `manipulability_ellipsoid.png` | 可操作性椭球图 | 可视化展示 |

---

## 👥 团队协作建议

### 代码规范
- 所有角度使用**弧度制**
- 坐标系遵循**右手定则**
- 函数命名采用**下划线风格**
- 添加必要的注释和文档字符串

### 扩展方向
1. **动力学完整实现**: 添加质量矩阵 M(q) 和科氏力 C(q,q̇)
2. **轨迹规划**: 基于雅可比的逆运动学求解
3. **力控制**: 阻抗控制、导纳控制实现
4. **碰撞检测**: 基于3D模型的碰撞检查
5. **仿真集成**: 与MuJoCo/PyBullet对接



## 📚 参考资料

1. Franka Emika FR3官方文档
2. 《机器人学导论》- John J. Craig
3. 《现代机器人学》- Kevin M. Lynch
4. MuJoCo物理引擎文档

---

## ⚠️ 注意事项

1. **参数准确性**: 当前的质量和质心参数为估计值，实际应用需进行参数辨识
2. **数值稳定性**: 接近奇异位形时注意数值稳定性问题
3. **单位统一**: 所有物理量使用国际单位制（SI）
4. **版本兼容**: 建议使用Python 3.7+以确保兼容性

---



---

**最后更新**: 2026年5月15日  
**维护者**: M1项目组













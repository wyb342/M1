# Franka FR3 机械臂运动学与动力学分析框架

## 📋 项目简介

本项目实现了Franka Emika FR3七自由度机械臂的完整运动学和动力学分析框架，包括正向运动学、雅可比矩阵计算（几何雅可比和解析雅可比）、奇异性分析、可操作性分析和重力项推导等功能模块。所有核心算法均通过MuJoCo物理引擎进行验证，确保数值准确性。

---

## 🗂️ 项目结构

- `M1/`
  - `dh_params.py` - DH参数定义（改进DH/Craig约定）
  - `kinematics.py` - 核心运动学函数（FK、几何雅可比、解析雅可比）
  - `gravity_term.py` - 重力项计算（基于MuJoCo引擎）
  - `test_fk.py` - 正向运动学验证（对比MuJoCo）
  - `test_jacobian.py` - 几何雅可比验证（有限差分法）
  - `test_analytical_jacobian.py` - 解析雅可比验证（欧拉角速率）
  - `plot_singularity.py` - 奇异性可视化分析
  - `visualize_manipulability.py` - 可操作性椭球可视化
  - `franka_fr3/` - FR3机器人模型文件
    - `assets/` - 3D模型文件(.stl, .obj)
    - `fr3.xml` - MuJoCo模型描述
    - `scene.xml` - 场景配置
  - `analytic_jacobian_example.csv` - 解析雅可比示例数据
  - `geometric_jacobian_example.csv` - 几何雅可比示例数据
  - `manipulability_comparison.csv` - 可操作性对比数据
  - `manipulability_ellipsoid.png` - 可操作性椭球图

---

## 🔧 核心模块说明

### 1. DH参数定义 (`dh_params.py`)

定义了FR3机器人的**改进DH参数（Modified DH / Craig convention）**：

**参数格式**: `[a_{i-1}, d_i, alpha_{i-1}, theta_offset]`

**关键参数：**
- `a`: 连杆长度（沿X轴的距离）
- `d`: 连杆偏距（沿Z轴的距离）
- `alpha`: 连杆扭转角（绕X轴的旋转）
- `theta_offset`: 关节角度偏移（FR3中均为0）

**DH变换顺序**：
1. 绕 Z_i 旋转 θ_i
2. 沿 Z_i 平移 d_i
3. 沿 X_i 平移 a_i
4. 绕 X_i 旋转 α_i

**注意**：末端执行器位姿返回的是 `fr3_link7` 坐标系，不包含法兰盘偏移。

### 2. 运动学核心 (`kinematics.py`)

提供以下核心函数：

#### `forward_kinematics(q)`
- **功能**: 计算正向运动学
- **输入**: 关节角度向量 q (7,) 弧度
- **输出**: 末端执行器位姿矩阵 T (4×4)
- **说明**: 返回 `fr3_link7` 的位姿（不含法兰盘），T 包含旋转矩阵和平移向量

#### `jacobian(q)`
- **功能**: 计算空间几何雅可比矩阵
- **输入**: 关节角度向量 q (7,)
- **输出**: 雅可比矩阵 J (6×7)
- **说明**: 
  - v = J @ dq，其中 v = [线速度; 角速度] 在世界坐标系下
  - 线速度部分: J_v,i = z_i × (p_ee - p_i)
  - 角速度部分: J_ω,i = z_i
  - 末端为 `fr3_link7`（不带法兰盘偏移）

#### `jacobian_analytic(q)`
- **功能**: 计算解析雅可比矩阵
- **输入**: 关节角度向量 q (7,)
- **输出**: 解析雅可比矩阵 J_analytic (6×7)
- **说明**: 
  - 使用ZYX欧拉角（yaw-pitch-roll）表示姿态
  - 输出形式: [线速度; 欧拉角速率] = J_analytic @ dq
  - 通过转换矩阵 B 将角速度映射到欧拉角速率
  - 处理万向锁情况（pitch ≈ ±π/2时使用伪逆）

#### `euler_zyx_from_rotation(R)`
- **功能**: 从旋转矩阵提取ZYX欧拉角
- **输入**: 旋转矩阵 R (3×3)
- **输出**: [yaw, pitch, roll] 弧度值
- **说明**: 旋转顺序 R = Rz(yaw) * Ry(pitch) * Rx(roll)，包含万向锁处理

#### `jacobian_singular_values(q)`
- **功能**: 计算雅可比矩阵的奇异值
- **输入**: 关节角度向量 q (7,)
- **输出**: 奇异值数组（降序排列）
- **用途**: 用于奇异性分析和可操作性评估

#### `condition_number(q)`
- **功能**: 计算雅可比矩阵条件数
- **输入**: 关节角度向量 q (7,)
- **输出**: 条件数标量（最大奇异值/最小奇异值）
- **说明**: 条件数越大越接近奇异位形，>100 需注意

### 3. 重力项计算 (`gravity_term.py`) ⭐新增

#### `GravityTerm` 类

基于MuJoCo物理引擎的重力项计算器，确保与动力学方程一致。

**初始化参数：**
- `xml_path`: MuJoCo模型文件路径（默认为 `franka_fr3/fr3.xml`）
- 自动从模型中提取质量和质心参数

主要方法：

##### `compute_gravity_term(q)`
- **功能**: 计算重力项 g(q)
- **输入**: 关节角度向量 q (7,)
- **输出**: 重力力矩向量 g_q (7,) N·m
- **原理**: 
  - 使用MuJoCo的 `mj_jac` 计算质心线速度雅可比
  - g_i = -Σ(m_j * g^T * J_v_j[:,i]) 对于所有连杆 j
  - 负号来自势能定义 V = -m*g^T*p

##### `compute_gravity_potential_energy(q)`
- **功能**: 计算系统重力势能
- **输入**: 关节角度向量 q (7,)
- **输出**: 势能标量 V (J)
- **公式**: V = -Σ(m_i * g^T * p_ci)

##### `compute_gravity_numerical_gradient(q, eps=1e-6)`
- **功能**: 数值梯度验证
- **输入**: 关节角度向量 q，差分步长 eps
- **输出**: 数值计算的重力项
- **用途**: 验证解析解正确性，误差应在 1e-6 量级

**物理模型：**
τ = M(q)q̈ + C(q,q̇)q̇ + g(q)

其中 g(q) 为重力产生的关节力矩，通过势能梯度或雅可比传递法计算。

**重要注意事项：**
- 直接使用MuJoCo的 `xipos` 获取世界坐标系下的质心位置
- 使用 `mj_jac` 计算线速度雅可比，避免DH链坐标转换问题
- 重力加速度向量 g = [0, 0, -9.81] m/s²（世界坐标系）

---

## 🚀 快速开始

### 环境要求
- Python >= 3.7
- 依赖库: numpy, matplotlib, mujoco

### 安装依赖
bash pip install numpy matplotlib mujoco


### 基本功能

#### 1. 正向运动学
#### 2. 雅可比矩阵计算
#### 3. 重力项计算
#### 4. 奇异性分析

---

## 📊 运行测试与可视化

### 运行所有测试

所有测试脚本都会与MuJoCo或有限差分法进行对比验证：

- **正向运动学测试**: `python test_fk.py` （对比MuJoCo，误差 < 1e-6）
- **几何雅可比测试**: `python test_jacobian.py` （有限差分验证，误差 < 1e-5）
- **解析雅可比测试**: `python test_analytical_jacobian.py` （欧拉角速率验证，误差 < 1e-5）
- **重力项测试**: `python gravity_term.py` （数值梯度验证，误差 ~1e-10）

### 生成可视化图表

#### 奇异性可视化
生成每个关节单独变化时的最小奇异值曲线，标记奇异风险区域。

#### 可操作性椭球
生成：
- 可操作度椭球3D可视化图（`manipulability_ellipsoid.png`）
- 多配置可操作度对比表（`manipulability_comparison.csv`）

---

## 📐 理论背景

### DH参数约定

采用**改进DH参数（Modified DH / Craig convention）**约定：
1. 绕 Z_i 旋转 θ_i
2. 沿 Z_i 平移 d_i
3. 沿 X_i 平移 a_i
4. 绕 X_i 旋转 α_i

### 雅可比矩阵

**几何雅可比**: 
- 线速度部分: J_v,i = z_i × (p_ee - p_i)
- 角速度部分: J_ω,i = z_i
- 表示末端速度 v = J @ dq（世界坐标系）

**解析雅可比**:
- 通过欧拉角映射矩阵 B 转换
- ω = B * [yaw_dot; pitch_dot; roll_dot]
- J_analytic = [J_v; B⁻¹ @ J_ω]
- 适用于任务空间控制（欧拉角反馈）

### 重力项推导

**方法一：势能梯度法**

$$
g(q) = \frac{\partial V}{\partial q}
$$

$$
V = -\sum m_i \cdot \mathbf{g}^T \cdot \mathbf{p}_{ci}
$$

**方法二：雅可比传递法（本实现采用）**

$$
g_i = -\sum_{j=i}^{n} m_j \mathbf{g}^T \mathbf{J}_{v,j}^{(:,i)}
$$

其中：
- $g_i$ 为第 $i$ 个关节的重力力矩分量
- $m_j$ 为第 $j$ 个连杆的质量
- $\mathbf{g}$ 为重力加速度向量 [0, 0, -9.81] m/s²
- $\mathbf{J}_{v,j}^{(:,i)}$ 为第 $j$ 个连杆质心相对于第 $i$ 个关节的线速度雅可比列向量
- $n$ 为机械臂总自由度数（此处为7）

**符号约定**：
- 势能定义为负：V = -m*g^T*p（重力向下时势能为负）
- 重力项为正：g(q) = ∂V/∂q
- MuJoCo中重力方向由 model.gravity 决定

### 可操作性分析

**Yoshikawa可操作度指标**:
$$
w = \sqrt{\det(J_v J_v^T)} = \sigma_1 \sigma_2 \sigma_3
$$

**可操作度椭球**:
- 主轴方向：JJ^T 的特征向量
- 主轴长度：奇异值 σ_i
- 椭球越接近球形，各向同性越好

**条件数**:
$$
\kappa = \frac{\sigma_{max}}{\sigma_{min}}
$$
- κ = 1: 完全各向同性
- κ → ∞: 接近奇异位形

---

## 🔍 常见问题

### Q1: 如何修改机器人参数？

编辑 `dh_params.py` 中的 `FR3_DH` 列表修改DH参数。重力相关参数（质量、质心）直接从MuJoCo模型 `franka_fr3/fr3.xml` 中读取，如需修改请编辑该XML文件。

### Q2: 数值验证误差较大怎么办？

检查：
1. 差分步长 eps 是否合适（推荐 1e-6 到 1e-8）
2. MuJoCo模型是否正确加载
3. 关节角度是否在合理范围内
4. 接近奇异位形时数值稳定性会下降

### Q3: 如何处理奇异位形？

当条件数 > 100 或最小奇异值 < 0.01 时：
1. 避免在该位形附近操作
2. 使用阻尼最小二乘法（Damped Least Squares）
3. 利用冗余自由度优化（FR3有7自由度）
4. 切换到解析雅可比避免欧拉角奇异

### Q4: 几何雅可比和解析雅可比有什么区别？

- **几何雅可比**: 输出 [线速度; 角速度]，适用于速度控制
- **解析雅可比**: 输出 [线速度; 欧拉角速率]，适用于姿态控制
- 两者通过矩阵 B 转换，在万向锁附近解析雅可比可能不稳定

### Q5: 为什么重力项计算要用MuJoCo？

- MuJoCo提供准确的质心位置和雅可比矩阵
- 避免DH链与MuJoCo坐标系不一致的问题
- 确保与动力学仿真的一致性
- 自动处理复杂的惯性参数

### Q6: 如何扩展到其他机器人？

1. 在 `dh_params.py` 中定义新的DH参数
2. 准备对应的MuJoCo XML模型文件
3. 调整 `gravity_term.py` 中的XML路径
4. 保持接口一致性即可复用所有函数

---

## 📝 输出文件说明

| 文件名 | 内容 | 用途 |
|--------|------|------|
| `analytic_jacobian_example.csv` | q=0时的解析雅可比矩阵(6×7) | 对比分析、调试 |
| `geometric_jacobian_example.csv` | q=0时的几何雅可比矩阵(6×7) | 对比分析、调试 |
| `manipulability_comparison.csv` | 多个配置的可操作度指标对比 | 性能评估、优化 |
| `manipulability_ellipsoid.png` | 可操作度椭球3D可视化图 | 展示、报告 |

---

## 👥 团队协作建议

### 代码规范
- 所有角度使用**弧度制**
- 坐标系遵循**右手定则**
- 函数命名采用**下划线风格**
- 添加必要的注释和文档字符串
- 单元测试必须通过MuJoCo或有限差分验证

### 扩展方向
1. **完整动力学实现**: 添加质量矩阵 M(q) 和科氏力矩阵 C(q,q̇)
2. **逆运动学求解**: 基于雅可比的数值IK或解析IK
3. **轨迹规划**: Cartesian space轨迹生成与跟踪
4. **力控制**: 阻抗控制、导纳控制实现
5. **碰撞检测**: 基于3D模型的碰撞检查
6. **参数辨识**: 基于实测数据的动力学参数辨识
7. **实时控制**: 与Franka控制接口对接

---

## 📚 参考资料

1. Franka Emika FR3官方文档与技术手册
2. 《机器人学导论》- John J. Craig (改进DH参数)
3. 《现代机器人学》- Kevin M. Lynch (雅可比理论)
4. "A Method for Registration of 3-D Shapes" - Yoshikawa (可操作性指标)
5. MuJoCo物理引擎官方文档 (https://mujoco.readthedocs.io/)
6. Franka Control Interface (FCI) 文档

---

## ⚠️ 注意事项

1. **参数准确性**: 当前的质量和质心参数从MuJoCo模型读取，实际应用需进行参数辨识以获得精确值
2. **数值稳定性**: 接近奇异位形时（条件数>100）注意数值稳定性问题，建议使用阻尼方法
3. **单位统一**: 所有物理量使用国际单位制（SI）：长度-m，质量-kg，角度-rad，力-N，力矩-N·m
4. **版本兼容**: 建议使用Python 3.7+和最新稳定版MuJoCo
5. **坐标系约定**: 
   - DH链坐标系与MuJoCo body frame可能不同
   - 重力项计算直接使用世界坐标系避免转换错误
6. **欧拉角奇异**: 解析雅可比在pitch≈±π/2时存在万向锁，此时应使用几何雅可比

---

**最后更新**: 2026年5月30日  
**维护者**: M1项目组

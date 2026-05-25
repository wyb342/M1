import mujoco
import numpy as np
from kinematics import forward_kinematics

# 加载模型（路径请根据你的项目结构调整）
model = mujoco.MjModel.from_xml_path("franka_fr3/fr3.xml")
data = mujoco.MjData(model)

# 关节限位（弧度）从模型读取
joint_limits = []
for i in range(7):
    joint_limits.append((model.jnt_range[i, 0], model.jnt_range[i, 1]))

np.random.seed(42)
errors_pos = []
errors_rot = []

for _ in range(100):
    q_rand = np.array([np.random.uniform(low, high) for low, high in joint_limits])
    data.qpos[:7] = q_rand
    mujoco.mj_forward(model, data)

    # MuJoCo 末端位姿 (取 fr3_hand)
    body_id = model.body("fr3_link7").id
    pos_muj = data.xpos[body_id].copy()
    rot_muj = data.xmat[body_id].reshape(3, 3).copy()

    # 你的 FK
    T_mine = forward_kinematics(q_rand)
    pos_mine = T_mine[:3, 3]
    rot_mine = T_mine[:3, :3]

    pos_err = np.linalg.norm(pos_mine - pos_muj)
    rot_err = np.linalg.norm(rot_mine - rot_muj, ord='fro')
    errors_pos.append(pos_err)
    errors_rot.append(rot_err)

print(f"位置误差: max={np.max(errors_pos):.2e}, mean={np.mean(errors_pos):.2e}")
print(f"姿态误差: max={np.max(errors_rot):.2e}, mean={np.mean(errors_rot):.2e}")

assert np.max(errors_pos) < 1e-6, "FK 位置误差超标"
assert np.max(errors_rot) < 1e-6, "FK 姿态误差超标"
print("✅ FK 验证通过")
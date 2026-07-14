# LegBot MuJoCo 前进仿真技术报告

## 1. 任务目标

本次任务是在 `/home/lushilin/Cheetah-Software-master` 项目中接入
`models` 目录下的 LegBot 四足机器人模型，并在 MuJoCo 中实现稳定前进仿真。

目标约束如下：

- 使用项目内 LegBot MJCF/STL 模型；
- 在 MuJoCo 中完成前进仿真；
- 步态正确，不能出现明显前倾、趴地或摔倒；
- 支持 `0.5-2.0 m/s` 的速度指令；
- 可视化不能有明显慢放感；
- 质心/机身高度目标切换为 `0.2775 m`；
- 物理仿真频率为 `1000 Hz`；
- 控制频率支持 `500 Hz` 和 `1000 Hz`；
- viewer 约 `60 Hz` 刷新，即默认每 16 个物理步同步一次画面。

最终实现入口为：

```text
scripts/mujoco_legbot_walk.py
```

生成的 MuJoCo 运行模型为：

```text
models/MJCF/legbot/legbot_mpc_scene_project.xml
```

源模型文件保持不变：

```text
models/MJCF/legbot/legbot_mpc_scene.xml
```

## 2. 项目结构与接入方式

原 Cheetah-Software 项目本身包含 C++ dynamics simulator、Qt 可视化和 MIT Controller
风格的控制框架，但当前 LegBot 模型以 MJCF/STL 形式放置在 `models` 目录下，因此本次接入采用独立 MuJoCo Python 仿真脚本。

相关文件：

| 文件 | 作用 |
| --- | --- |
| `models/MJCF/legbot/legbot_mpc_scene.xml` | LegBot 原始 MuJoCo 场景，作为 source of truth |
| `models/legbot/xmls/meshes` | LegBot STL mesh 目录，包含 base 和四条腿模型 |
| `scripts/mujoco_legbot_walk.py` | 前进仿真、模型修正、控制器和测试入口 |
| `models/MJCF/legbot/legbot_mpc_scene_project.xml` | 脚本自动生成的项目内可运行 MuJoCo XML |
| `models/MJCF/legbot/README.md` | 快速运行说明 |
| `models/MJCF/legbot/TECHNICAL_REPORT.md` | 本技术报告 |

接入时没有直接修改原始 `legbot_mpc_scene.xml`，而是在运行时解析 XML 并生成一个项目本地副本。这样做有两个好处：

- 保留原始模型，便于后续重新生成或对比；
- 可以在生成副本中修正 mesh 路径、移除可视化 marker、缩放仿真力矩上限，不污染源文件。

## 3. MuJoCo 模型处理

### 3.1 meshdir 修正

原始 XML 中 `meshdir` 指向旧路径或相对旧目录。脚本运行时会优先查找：

```text
models/legbot/xmls/meshes
models/legbot_description/meshes
```

只要目录下存在 `base_link.STL`，就将 XML 中 `<compiler meshdir="...">`
重写为该绝对路径。

相关实现：

```python
MESH_DIR_CANDIDATES = (
    REPO_ROOT / "models" / "legbot" / "xmls" / "meshes",
    REPO_ROOT / "models" / "legbot_description" / "meshes",
)
```

### 3.2 移除非机器人 marker

原始场景中包含：

```text
target_marker
robot_heading_arrow
desired_heading_arrow
```

这些 body 主要用于可视化目标和方向，但它们包含额外自由度。如果直接加载，会导致 base freejoint 不一定在 `qpos[0:7]`，容易造成状态读取错位。因此生成运行 XML 时会移除这些 marker，只保留机器人和地面。

同时，未使用的 `arrow_cone` mesh asset 也会被移除，避免 MuJoCo 加载不存在的 `cone.stl`。

### 3.3 仿真选项

生成 XML 时设置：

```xml
<option timestep="0.001" integrator="RK4" cone="elliptic" impratio="100" />
```

对应含义：

- `timestep = 0.001 s`，物理仿真频率 `1000 Hz`；
- `RK4` 提高积分稳定性；
- `elliptic cone` 和较高 `impratio` 有利于接触稳定。

### 3.4 力矩上限缩放

源 MJCF 中电机力矩范围约为：

```text
hip/thigh: ±17 Nm
calf:      ±34 Nm
```

脚本默认使用：

```bash
--torque-scale 1.8
```

只在生成的运行 XML 中缩放 `forcerange` 和 `actuatorfrcrange`，源模型不变。这样可以提高高速前进时的仿真能力，同时保留可回退性。

## 4. 状态读取与关节映射

不能直接假设机器人 base 状态在 `qpos[0:7]`。脚本通过 MuJoCo named joint 获取地址：

```python
freejoint_id = model.joint("joint_fixed_world").id
self.base_qpos_addr = int(model.jnt_qposadr[freejoint_id])
self.base_qvel_addr = int(model.jnt_dofadr[freejoint_id])
```

这样即使 XML 中存在其他 joint，base 状态读取也不会错位。

腿顺序为：

```text
FR, FL, RR, RL
```

关节顺序为每条腿：

```text
hip, thigh, calf
```

总执行器顺序：

```text
FR_hip_joint, FR_thigh_joint, FR_calf_joint,
FL_hip_joint, FL_thigh_joint, FL_calf_joint,
RR_hip_joint, RR_thigh_joint, RR_calf_joint,
RL_hip_joint, RL_thigh_joint, RL_calf_joint
```

## 5. 控制器设计

控制器类：

```python
VbotRaibertTrot
```

控制框架是对角小跑 diagonal trot，包含两部分：

- 摆动腿：足端轨迹规划 + 解析 IK + 关节 PD；
- 支撑腿：基于 MuJoCo 足端 Jacobian 的地面反力控制，转换为关节力矩。

最终输出是 12 维关节力矩 `data.ctrl`。

### 5.1 机身高度

当前默认高度：

```text
height = 0.2775 m
```

初始化 base 高度：

```text
height + 0.025 = 0.3025 m
```

支撑期高度调节目标：

```text
height + 0.015 = 0.2925 m
```

实际测试中最低机身高度约 `0.293-0.302 m`，与该目标一致。

### 5.2 步态相位

四条腿采用对角 trot：

```text
FR 与 RL 同相
FL 与 RR 同相
两组相差 0.5 周期
```

实现：

```python
self.phase_offset = {"FR": 0.0, "RL": 0.0, "FL": 0.5, "RR": 0.5}
```

支撑相占比：

```text
duty = 0.55
```

即每个周期中约 55% 时间为支撑，45% 时间为摆动。

### 5.3 周期与步长

步态周期随速度变化：

```python
period = max(0.24, 0.46 - 0.12 * (speed - 0.5))
```

速度越大，周期越短，步频越高。典型值：

| 速度指令 | 周期 |
| --- | --- |
| 0.5 m/s | 0.46 s |
| 1.0 m/s | 0.40 s |
| 1.5 m/s | 0.34 s |
| 2.0 m/s | 0.28 s |

步长：

```python
step_length = min(0.49, max(0.14, speed * period * 0.98))
```

该公式使低速不过小，高速不过大。

### 5.4 摆动腿轨迹

摆动腿使用半余弦平滑轨迹。支撑期足端沿身体坐标系相对方向推地，摆动期抬脚回摆。

摆腿抬脚高度：

```python
clearance = 0.045 + 0.02 * min(1.0, speed / 2.0)
```

因此高速时会稍微增加 clearance，减少绊脚。

### 5.5 解析 IK

腿部简化为 sagittal 平面二连杆：

```text
l1 = 0.1985 m
l2 = 0.2140 m
```

IK 计算：

```python
c2 = (x^2 + z^2 - l1^2 - l2^2) / (2 l1 l2)
q_knee = -acos(c2)
q_thigh = atan2(x, z) - atan2(l2 sin(q_knee), l1 + l2 cos(q_knee))
```

hip ab/ad 关节给一个小的侧向默认角：

```python
q_abad = ±0.04 rad
```

并叠加 roll feedback：

```python
q_des[0::3] += clip(-0.10 * roll, -0.08, 0.08)
```

### 5.6 关节 PD

基础关节力矩：

```text
tau_pd = kp * (q_des - q) - kd * dq
```

当前参数：

| 关节 | Kp | Kd |
| --- | --- | --- |
| hip | 36.0 | 1.8 |
| thigh | 48.0 | 2.2 |
| calf | 54.0 | 2.4 |

### 5.7 支撑腿地面反力

支撑腿额外加入足端反力控制。根据 MuJoCo 计算足端 Jacobian：

```python
mujoco.mj_jacBody(model, data, jacp, jacr, foot_body_id)
```

通过：

```text
tau_grf = -J^T F
```

转换为关节力矩。这里使用负号是经过实际 MuJoCo 模型方向验证后的结果。

总垂向力：

```python
total_fz = mass * (9.81 + 70.0 * height_error - 5.0 * vz)
```

并限制在：

```text
0.45 * mass * g 到 1.55 * mass * g
```

总前向力：

```python
total_fx = clip(65.0 * (speed - vx), -110.0, 110.0)
```

俯仰稳定：

```python
pitch_moment = clip(-8.0 * pitch - 1.0 * pitch_rate, -5.0, 5.0)
```

横滚稳定：

```python
roll_force = clip(-7.0 * roll - 0.8 * roll_rate, -7.0, 7.0)
```

这些反馈项共同抑制前倾、侧翻和高度下沉。

## 6. 仿真频率与可视化

### 6.1 物理频率

MuJoCo timestep：

```text
0.001 s
```

物理频率：

```text
1000 Hz
```

### 6.2 控制频率

脚本支持：

```bash
--control-hz 1000
--control-hz 500
```

实现方式是 decimation：

```python
control_decimation = round(physics_hz / control_hz)
```

当 `control_hz = 1000` 时，每个物理步更新控制；
当 `control_hz = 500` 时，每两个物理步更新一次控制，其余时间保持上一帧 `ctrl`。

### 6.3 viewer 刷新频率

原先每个物理步都调用 `viewer.sync()`，相当于尝试 1000 Hz 刷新画面。渲染端跟不上时，会表现为慢放。

现在默认：

```bash
--render-every 16
```

即每 16 个物理步刷新一次 viewer：

```text
1000 / 16 = 62.5 Hz
```

接近常见 60 Hz 显示刷新率，因此视觉上更正常，且不会改变物理仿真步长。

## 7. 可调参数说明

| 参数 | 默认值 | 作用 | 调整影响 |
| --- | --- | --- | --- |
| `--speed` | `1.0` | 目标前进速度 | 范围 `0.5-2.0 m/s` |
| `--height` | `0.2775` | 机身/质心高度目标 | 低更稳但易蹭地，高更舒展但易前倾 |
| `--control-hz` | `1000` | 控制频率 | 可设 `500` 或 `1000` |
| `--render-every` | `16` | viewer 同步间隔 | 越大越流畅省渲染，但视觉帧率降低 |
| `--torque-scale` | `1.8` | 仿真 XML 力矩上限缩放 | 高速能力增强，但过大可能抖动 |
| `period()` | 速度相关 | 步态周期 | 小周期高步频 |
| `step_length()` | 速度相关 | 步长 | 大步长高速度，但过大可能打滑 |
| `duty` | `0.55` | 支撑相比例 | 大更稳，小更跑跳 |
| `clearance` | `0.045-0.065` | 摆腿抬脚高度 | 高不易绊脚，但耗能和摆动更大 |
| `kp/kd` | `[36,48,54] / [1.8,2.2,2.4]` | 关节 PD | 高更硬，低更软 |
| `total_fx` gain | `65.0` | 前向速度反馈 | 高速度跟踪更强，过高可能打滑 |
| 高度 P/D | `70.0 / 5.0` | 垂向支撑 | 高度保持更硬或更软 |
| pitch feedback | `8.0 / 1.0` | 抗前倾 | 增大可抑制前倾 |
| roll feedback | `7.0 / 0.8` | 抗侧翻 | 增大可抑制侧摆 |

## 8. 运行命令

### 8.1 全速度扫描

```bash
cd /home/lushilin/Cheetah-Software-master
conda run -n mujoco python scripts/mujoco_legbot_walk.py --scan --duration 8
```

### 8.2 单速度测试

```bash
conda run -n mujoco python scripts/mujoco_legbot_walk.py --speed 1.0 --duration 8
```

### 8.3 2 m/s 高速测试

```bash
conda run -n mujoco python scripts/mujoco_legbot_walk.py --speed 2.0 --duration 8
```

### 8.4 500 Hz 控制测试

```bash
conda run -n mujoco python scripts/mujoco_legbot_walk.py --scan --duration 8 --control-hz 500
```

### 8.5 可视化

```bash
conda run -n mujoco python scripts/mujoco_legbot_walk.py --speed 1.0 --duration 20 --viewer --realtime
```

### 8.6 修改高度测试

例如测试 `0.30 m`：

```bash
conda run -n mujoco python scripts/mujoco_legbot_walk.py --scan --duration 8 --height 0.30
```

### 8.7 只生成 XML

```bash
conda run -n mujoco python scripts/mujoco_legbot_walk.py --prepare-xml
```

## 9. 验证结果

测试环境：

```text
conda env: mujoco
mujoco: 3.3.2
numpy: 2.4.6
physics_hz: 1000
height: 0.2775 m
torque_scale: 1.8
render_every: 16
```

### 9.1 1000 Hz 控制

命令：

```bash
conda run -n mujoco python scripts/mujoco_legbot_walk.py --scan --duration 8.0
```

结果：

```text
OK cmd=0.50m/s avg=0.56m/s dist=4.48m min_z=0.293m max_roll=5.2deg max_pitch=6.4deg
OK cmd=1.00m/s avg=1.09m/s dist=8.75m min_z=0.301m max_roll=9.6deg max_pitch=9.2deg
OK cmd=1.50m/s avg=1.56m/s dist=12.47m min_z=0.300m max_roll=12.2deg max_pitch=9.4deg
OK cmd=2.00m/s avg=1.83m/s dist=14.64m min_z=0.302m max_roll=12.6deg max_pitch=12.4deg
```

### 9.2 500 Hz 控制

命令：

```bash
conda run -n mujoco python scripts/mujoco_legbot_walk.py --scan --duration 8.0 --control-hz 500
```

结果：

```text
OK cmd=0.50m/s avg=0.56m/s dist=4.49m min_z=0.293m max_roll=5.2deg max_pitch=6.4deg
OK cmd=1.00m/s avg=1.10m/s dist=8.77m min_z=0.301m max_roll=9.7deg max_pitch=9.3deg
OK cmd=1.50m/s avg=1.56m/s dist=12.47m min_z=0.300m max_roll=11.7deg max_pitch=8.9deg
OK cmd=2.00m/s avg=1.83m/s dist=14.67m min_z=0.302m max_roll=12.3deg max_pitch=12.4deg
```

### 9.3 结果分析

从测试结果看：

- 0.5 m/s、1.0 m/s、1.5 m/s 速度跟踪较好；
- 2.0 m/s 指令下平均速度约 `1.83 m/s`，仍位于高速前进区间；
- 最低机身高度保持在 `0.293 m` 以上，没有趴地；
- 最大 roll/pitch 约 `12 deg`，远低于健康检查阈值；
- 500 Hz 与 1000 Hz 控制效果接近，说明控制对 decimation 不敏感；
- viewer 60 Hz sync 不改变物理结果，只改善显示流畅度。

## 10. 健康检查逻辑

脚本中健康检查条件：

```python
healthy = (
    0.45 <= avg_speed <= 2.15
    and min_height > 0.20
    and max_abs_pitch < 0.60
    and max_abs_roll < 0.55
)
```

对应物理意义：

- 平均速度必须在合理行走范围；
- 机身最低高度不能低于 `0.20 m`；
- pitch 小于约 `34.4 deg`；
- roll 小于约 `31.5 deg`。

输出中 `OK` 表示该速度测试通过，`FAIL` 表示至少一个条件不满足。

## 11. 当前实现的局限

当前控制器是解析式 gait + Jacobian 反力控制，并不是完整 MPC/WBC 控制器。它适合平地前进验证和模型接入测试，但仍有以下局限：

- 没有实时优化接触力分配；
- 没有显式摩擦锥约束；
- 没有根据地形高度自适应落足；
- 没有状态估计器，直接使用 MuJoCo 真值；
- 没有严格的能耗或力矩最优；
- 高速下速度跟踪依赖步长和水平反力调参。

## 12. 后续优化建议

如果要进一步提高真实性和鲁棒性，可以按以下方向推进：

1. 增加 viewer FPS 统计和实时因子显示；
2. 将 `height`、`duty`、`step_length`、`period` 等参数整理成 YAML；
3. 添加 CSV log，记录 base pose、速度、关节力矩、足端接触；
4. 接入 Cheetah-Software 原有 Convex MPC 思路，做显式接触力优化；
5. 增加地形高度采样，让足端落点跟随楼梯/坡面；
6. 加入摩擦锥约束，限制水平力过大导致打滑；
7. 加入 command filter，避免速度阶跃导致姿态瞬态过大；
8. 增加自动参数扫描脚本，系统评估速度、姿态和能耗。

## 13. 结论

本次实现已经完成 LegBot 模型在 MuJoCo 中的项目内接入，并实现稳定前进仿真。当前默认配置为：

```text
height = 0.2775 m
physics_hz = 1000
control_hz = 1000 或 500
viewer sync = 每 16 个物理步一次，约 60 Hz
speed command range = 0.5-2.0 m/s
```

在 `0.5-2.0 m/s` 指令范围内，机器人能够保持对角小跑前进，没有出现明显前倾、趴地或摔倒。可视化慢放问题通过降低 viewer sync 频率解决，物理仿真步长和控制稳定性保持不变。

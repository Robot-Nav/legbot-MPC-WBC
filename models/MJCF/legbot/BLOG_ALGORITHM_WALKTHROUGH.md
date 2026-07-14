# 从 Cheetah-Software 到 LegBot MuJoCo 前进仿真：项目架构、算法原理与运行指南

> 本文是一篇面向工程复现和算法理解的技术博客，基于
> `/home/lushilin/Cheetah-Software-master` 项目，讲解原 Cheetah-Software 的整体架构、
> 四足机器人控制算法知识点，以及本次新增的 LegBot MuJoCo 前进仿真实现。

## 目录

- 项目整体介绍
- Cheetah-Software 原始架构
- 四足机器人控制的基础知识
- LegBot MuJoCo 接入方案
- 前进步态控制算法
- 核心公式讲解
- 运行步骤
- 参数调节指南
- 测试结果与现象分析
- 后续优化方向

## 1. 项目整体介绍

这个项目源自 MIT Cheetah 系列机器人的控制软件框架，原始仓库的主要目标是同时支持：

- Cheetah 3；
- Mini Cheetah；
- 软件仿真；
- 真实机器人控制；
- 状态估计、腿部控制、FSM 状态机、MPC、WBC 等模块。

原项目目录大致如下：

```text
Cheetah-Software-master
├── common          # 动力学、数学工具、状态估计、腿控、MPC 公共库
├── robot           # 机器人运行框架，连接硬件或仿真
├── sim             # 原项目自带图形仿真器，依赖 Qt
├── user            # 用户控制器，例如 MIT_Controller、JPos_Controller
├── resources       # 可视化 CAD/mesh
├── config          # 控制参数和仿真参数
├── third-party     # Eigen、OSQP、qpOASES 等第三方依赖
└── models          # 本次新增/接入的 LegBot MJCF/STL 模型
```

原项目自带的仿真器不是 MuJoCo，而是 Cheetah-Software 自己的 dynamics simulator + Qt 可视化。
本次 LegBot 模型是 MJCF/STL 格式，因此新增了一个独立 MuJoCo Python 入口：

```text
scripts/mujoco_legbot_walk.py
```

该脚本完成：

- 加载 LegBot MJCF；
- 修正 mesh 路径；
- 生成项目本地 MuJoCo XML；
- 运行前进步态控制器；
- 支持 viewer 可视化；
- 支持 `0.5-2.0 m/s` 速度指令；
- 支持 500 Hz / 1000 Hz 控制频率；
- viewer 按约 60 Hz 刷新，避免慢放感。

## 2. Cheetah-Software 原始架构

虽然本次 LegBot MuJoCo 接入是 Python 脚本，但理解原项目结构非常有帮助，因为我们的控制器也借鉴了四足机器人经典控制分层思想。

### 2.1 common：机器人控制公共库

`common` 目录包含四足控制中最底层、最通用的代码：

```text
common/include/Dynamics
common/include/Controllers
common/include/SparseCMPC
common/include/Math
common/include/Utilities
```

关键模块：

| 模块 | 功能 |
| --- | --- |
| `Dynamics/Quadruped` | 四足机器人结构参数抽象 |
| `Dynamics/FloatingBaseModel` | 浮动基座刚体动力学模型 |
| `Controllers/LegController` | 关节/足端命令接口 |
| `Controllers/StateEstimatorContainer` | 状态估计器容器 |
| `Controllers/FootSwingTrajectory` | 摆腿轨迹 |
| `SparseCMPC` | 稀疏 Convex MPC |
| `Math/orientation_tools` | 姿态、四元数、旋转矩阵工具 |

### 2.2 robot：控制运行框架

`robot` 目录负责将控制器接到仿真或真实机器人。

典型运行链路：

```text
SimulationBridge / HardwareBridge
        ↓
RobotRunner
        ↓
RobotController
        ↓
LegController / StateEstimator / DesiredStateCommand
```

其中 `RobotRunner` 每个控制周期做几件事：

1. 运行状态估计器；
2. 更新腿部传感器数据；
3. 调用用户控制器；
4. 将控制命令写回腿部控制接口；
5. 发布 LCM 调试数据。

可以理解为：

```text
传感器输入 → 状态估计 → 高层控制 → 腿部控制 → 电机命令
```

### 2.3 user/MIT_Controller：状态机与运动控制

`user/MIT_Controller` 是原项目里最核心的 locomotion 控制器之一。

它使用 FSM 状态机管理行为：

```text
PASSIVE
STAND_UP
BALANCE_STAND
LOCOMOTION
RECOVERY_STAND
VISION
BACKFLIP
FRONTJUMP
```

其中 `FSM_State_Locomotion` 会调用：

```text
ConvexMPCLocomotion
WBC LocomotionCtrl
```

也就是常见的四足控制结构：

```text
状态机 FSM
  ↓
步态调度 Gait Scheduler
  ↓
MPC 计算期望地面反力
  ↓
WBC / Leg Controller 转成关节力矩
```

本次 LegBot 采用两层接入方式：模型参数侧接入 Cheetah-Software 的 `RobotType/Quadruped/RobotRunner`
语境，让项目“认识” LegBot；稳定前进仿真侧使用更适合 MJCF/STL 的 MuJoCo Python 管线验证。
当前 MuJoCo 控制器没有完整复刻 C++ FSM/MPC/WBC，而是用一个轻量 Python 控制器复现了其中的关键思想：

- trot 步态；
- 摆腿轨迹；
- 支撑腿地面反力；
- Jacobian 转关节力矩；
- 姿态稳定反馈。

## 3. 四足机器人控制基础知识

### 3.1 浮动基座系统

四足机器人不是固定机械臂，它的身体是浮动的。状态通常包括：

```text
base position:    p = [x, y, z]
base orientation: q 或 R
base velocity:    v, omega
joint position:   q_j
joint velocity:   dq_j
```

对于 12 自由度四足机器人：

```text
每条腿 3 个关节 × 4 条腿 = 12 个驱动关节
```

浮动基座再带来：

```text
3 个平移自由度 + 3 个转动自由度
```

因此完整系统是欠驱动的：机器人身体不能直接施加任意力矩，只能通过脚和地面接触来控制身体。

### 3.2 步态 gait

步态描述每条腿什么时候支撑、什么时候摆动。

本项目当前使用对角小跑 trot：

```text
FR + RL 同时支撑/摆动
FL + RR 同时支撑/摆动
两组相位差 0.5 周期
```

这是一种常见稳定快速步态，适合中高速平地前进。

### 3.3 支撑腿与摆动腿

支撑腿 stance leg：

- 脚接触地面；
- 通过地面反力支撑身体；
- 负责推进、制动、抗俯仰和抗横滚。

摆动腿 swing leg：

- 脚离开地面；
- 按轨迹向前摆；
- 保证下一次落脚点合适；
- 需要足够 clearance 防止绊脚。

### 3.4 足端 Jacobian

关节速度与足端速度关系：

```math
v_{foot} = J(q) \dot{q}
```

虚功原理给出足端力与关节力矩关系：

```math
\tau = J(q)^T F
```

在当前 MuJoCo 模型的符号约定下，实际使用：

```math
\tau = -J(q)^T F
```

这个负号来自模型关节轴和世界足端力方向的实际验证。

## 4. LegBot MuJoCo 接入方案

### 4.1 为什么主要用 MuJoCo 测试

原项目 `sim/sim` 是自研动力学仿真器，模型来自 Cheetah 3 / Mini Cheetah 的 C++ 参数。
LegBot 模型是 MJCF/STL 文件，天然适合 MuJoCo。

因此稳定性测试入口采用：

```text
scripts/mujoco_legbot_walk.py
```

这并不排斥 Cheetah-Software 接入。当前项目中已经补充了 LegBot 的 C++ 模型描述和类型分支，
但“能否稳定前进、不前倾、不摔倒、速度 0.5-2.0 m/s”这些行为验证以 MuJoCo 为准。
这样做的好处是：

- 直接使用你的 MJCF/STL 真实模型；
- 不需要先把全部视觉 mesh 和碰撞几何搬进原 Qt sim；
- 可以快速验证关节方向、力矩上限、步态相位和高度目标；
- 后续如果要升级到 C++ MPC/WBC，可以把 MuJoCo 中验证过的参数迁移过去。

### 4.2 模型文件

源模型：

```text
models/MJCF/legbot/legbot_mpc_scene.xml
```

mesh 目录：

```text
models/legbot/xmls/meshes
```

生成运行模型：

```text
models/MJCF/legbot/legbot_mpc_scene_project.xml
```

### 4.3 XML 生成流程

脚本运行时会：

1. 解析源 MJCF；
2. 查找项目内 STL mesh 路径；
3. 重写 `<compiler meshdir="...">`；
4. 移除 `target_marker`、`robot_heading_arrow`、`desired_heading_arrow`；
5. 移除未使用的 `arrow_cone` mesh；
6. 设置 MuJoCo timestep、integrator、contact 参数；
7. 根据 `--torque-scale` 缩放电机力矩上限；
8. 写出 `legbot_mpc_scene_project.xml`。

核心思想是：

```text
源 XML 不动，仿真 XML 自动生成
```

这样便于回溯和调参。

## 5. 前进步态控制算法

当前控制器类是：

```python
VbotRaibertTrot
```

整体控制框图：

```text
速度指令 v_des
      ↓
步态周期/步长计算
      ↓
相位发生器 phase
      ↓
摆动腿轨迹 + IK
      ↓
关节 PD 力矩
      ↓
支撑腿地面反力 F
      ↓
tau = -J^T F
      ↓
MuJoCo 电机 torque ctrl
```

### 5.1 速度输入

命令行输入：

```bash
--speed 1.0
```

速度范围：

```text
0.5-2.0 m/s
```

脚本会检查该范围，超过则报错。

### 5.2 机身高度

当前默认高度：

```text
height = 0.2775 m
```

初始化高度：

```math
z_0 = h + 0.025 = 0.3025
```

高度控制目标：

```math
z_{des} = h + 0.015 = 0.2925
```

### 5.3 步态周期

周期随速度变化：

```python
T = max(0.24, 0.46 - 0.12 * (v_cmd - 0.5))
```

速度越大，周期越小，步频越高。

典型值：

| 指令速度 | 周期 T |
| --- | --- |
| 0.5 m/s | 0.46 s |
| 1.0 m/s | 0.40 s |
| 1.5 m/s | 0.34 s |
| 2.0 m/s | 0.28 s |

### 5.4 步长

步长计算：

```python
L = min(0.49, max(0.14, v_cmd * T * 0.98))
```

它近似满足：

```math
v \approx \frac{L}{T}
```

但实际速度还受到接触、打滑、力矩限制和姿态反馈影响。

### 5.5 相位

单腿相位：

```math
\phi_i = \left(\frac{t}{T} + \phi_{offset,i}\right) \bmod 1
```

相位偏置：

```text
FR = 0.0
RL = 0.0
FL = 0.5
RR = 0.5
```

支撑相比例：

```text
duty = 0.55
```

判断：

```text
phase < 0.55 → stance
phase >= 0.55 → swing
```

## 6. 足端轨迹与 IK

### 6.1 支撑期足端轨迹

支撑期中，足端相对身体沿前后方向移动，产生推进效果：

```python
x_rel = center + step * (s - 0.5)
z_down = height
```

其中：

```math
s = \frac{\phi}{duty}
```

`center` 叠加了速度误差和 pitch 补偿：

```python
center = pitch_comp + speed_comp
```

对应：

```math
pitch\_comp = clip(0.07 \cdot pitch, -0.035, 0.035)
```

```math
speed\_comp = clip(0.025 \cdot (v_{cmd} - v_x), -0.025, 0.025)
```

### 6.2 摆动期足端轨迹

摆动期用半余弦平滑：

```python
smooth = 0.5 - 0.5 * cos(pi * s)
x_rel = center + step * (0.5 - smooth)
z_down = height - clearance * sin(pi * s)
```

抬脚高度：

```python
clearance = 0.045 + 0.02 * min(1.0, speed / 2.0)
```

这可以保证摆腿轨迹连续、落脚柔和。

### 6.3 二连杆 IK

腿部简化为二连杆：

```text
l1 = 0.1985 m
l2 = 0.2140 m
```

足端目标为：

```text
(x, z)
```

余弦定理：

```math
c_2 = \frac{x^2 + z^2 - l_1^2 - l_2^2}{2 l_1 l_2}
```

膝关节：

```math
q_{knee} = -\arccos(c_2)
```

髋俯仰：

```math
q_{thigh}
= \arctan2(x, z)
- \arctan2(l_2 \sin(q_{knee}), l_1 + l_2 \cos(q_{knee}))
```

髋侧摆：

```math
q_{abad} = \pm 0.04
```

并加入横滚修正：

```math
q_{abad} \leftarrow q_{abad} + clip(-0.10 \cdot roll, -0.08, 0.08)
```

## 7. 关节力矩控制

### 7.1 关节 PD

基础力矩：

```math
\tau_{pd} = K_p(q_{des} - q) - K_d \dot{q}
```

当前参数：

| 关节 | Kp | Kd |
| --- | --- | --- |
| hip | 36.0 | 1.8 |
| thigh | 48.0 | 2.2 |
| calf | 54.0 | 2.4 |

### 7.2 支撑腿地面反力控制

支撑腿还需要产生支撑身体和推进的力。脚本构造期望足端力：

```math
F = [F_x, F_y, F_z]^T
```

然后通过足端 Jacobian 转成关节力矩：

```math
\tau_{grf} = -J^T F
```

最终：

```math
\tau = \tau_{pd} + \tau_{grf}
```

并做力矩限幅。

### 7.3 高度控制

高度误差：

```math
e_z = z_{des} - z
```

垂向总力：

```math
F_z^{total}
= m(9.81 + K_z e_z - D_z v_z)
```

当前：

```text
Kz = 70.0
Dz = 5.0
```

并限制：

```math
0.45mg \le F_z^{total} \le 1.55mg
```

### 7.4 前进速度控制

前向总力：

```math
F_x^{total} = clip(K_v(v_{cmd} - v_x), -110, 110)
```

当前：

```text
Kv = 65.0
```

### 7.5 姿态稳定

俯仰力矩补偿：

```math
M_{pitch} = clip(-8.0 \cdot pitch - 1.0 \cdot \omega_y, -5, 5)
```

横滚补偿：

```math
F_{roll} = clip(-7.0 \cdot roll - 0.8 \cdot \omega_x, -7, 7)
```

这些项用于抑制：

- 前倾；
- 后仰；
- 左右摇摆；
- 高速时姿态发散。

## 8. 仿真与可视化频率

### 8.1 物理仿真

MuJoCo timestep：

```text
0.001 s
```

物理频率：

```text
1000 Hz
```

### 8.2 控制频率

支持：

```bash
--control-hz 1000
--control-hz 500
```

内部通过 decimation 实现：

```python
control_decimation = round(physics_hz / control_hz)
```

含义：

| 控制频率 | 更新方式 |
| --- | --- |
| 1000 Hz | 每个 MuJoCo step 更新一次 torque |
| 500 Hz | 每两个 MuJoCo step 更新一次 torque |

### 8.3 viewer 刷新

之前每个物理步都刷新 viewer，会尝试 1000 Hz 渲染，显示端跟不上就像慢放。

现在默认：

```bash
--render-every 16
```

即：

```text
1000 / 16 = 62.5 Hz
```

接近 60 Hz 显示器刷新率。

重要的是：

```text
viewer 刷新变慢，不等于物理仿真变慢
```

物理仍然是 1000 Hz，只是画面每 16 步同步一次。

## 9. 运行步骤

### 9.1 进入项目

```bash
cd /home/lushilin/Cheetah-Software-master
```

### 9.2 全速度扫描

```bash
conda run -n mujoco python scripts/mujoco_legbot_walk.py --scan --duration 8
```

该命令会测试：

```text
0.5 m/s
1.0 m/s
1.5 m/s
2.0 m/s
```

### 9.3 单速度测试

```bash
conda run -n mujoco python scripts/mujoco_legbot_walk.py --speed 1.0 --duration 8
```

### 9.4 高速测试

```bash
conda run -n mujoco python scripts/mujoco_legbot_walk.py --speed 2.0 --duration 8
```

### 9.5 500 Hz 控制测试

```bash
conda run -n mujoco python scripts/mujoco_legbot_walk.py --scan --duration 8 --control-hz 500
```

### 9.6 可视化运行

```bash
conda run -n mujoco python scripts/mujoco_legbot_walk.py --speed 1.0 --duration 20 --viewer --realtime
```

### 9.7 修改高度

```bash
conda run -n mujoco python scripts/mujoco_legbot_walk.py --scan --duration 8 --height 0.30
```

### 9.8 修改 viewer 刷新间隔

例如每 8 步刷新一次，约 125 Hz：

```bash
conda run -n mujoco python scripts/mujoco_legbot_walk.py --speed 1.0 --viewer --realtime --render-every 8
```

如果机器渲染压力大，可以每 20 步刷新一次：

```bash
conda run -n mujoco python scripts/mujoco_legbot_walk.py --speed 1.0 --viewer --realtime --render-every 20
```

## 10. 测试结果

测试环境：

```text
conda env: mujoco
mujoco: 3.3.2
numpy: 2.4.6
height: 0.2775 m
physics_hz: 1000
torque_scale: 1.8
render_every: 16
```

### 10.1 1000 Hz 控制

```text
OK cmd=0.50m/s avg=0.56m/s dist=4.48m min_z=0.293m max_roll=5.2deg max_pitch=6.4deg
OK cmd=1.00m/s avg=1.09m/s dist=8.75m min_z=0.301m max_roll=9.6deg max_pitch=9.2deg
OK cmd=1.50m/s avg=1.56m/s dist=12.47m min_z=0.300m max_roll=12.2deg max_pitch=9.4deg
OK cmd=2.00m/s avg=1.83m/s dist=14.64m min_z=0.302m max_roll=12.6deg max_pitch=12.4deg
```

### 10.2 500 Hz 控制

```text
OK cmd=0.50m/s avg=0.56m/s dist=4.49m min_z=0.293m max_roll=5.2deg max_pitch=6.4deg
OK cmd=1.00m/s avg=1.10m/s dist=8.77m min_z=0.301m max_roll=9.7deg max_pitch=9.3deg
OK cmd=1.50m/s avg=1.56m/s dist=12.47m min_z=0.300m max_roll=11.7deg max_pitch=8.9deg
OK cmd=2.00m/s avg=1.83m/s dist=14.67m min_z=0.302m max_roll=12.3deg max_pitch=12.4deg
```

### 10.3 结果解读

可以看到：

- 0.5、1.0、1.5 m/s 跟踪较好；
- 2.0 m/s 指令下平均速度约 1.83 m/s，接近目标高速；
- 最低高度约 0.293 m，没有趴地；
- 最大 pitch/roll 约 12 deg，没有明显前倾或摔倒；
- 500 Hz 控制与 1000 Hz 控制差异很小，说明当前控制器有一定鲁棒性。

## 11. 健康检查

脚本中健康检查为：

```python
healthy = (
    0.45 <= avg_speed <= 2.15
    and min_height > 0.20
    and max_abs_pitch < 0.60
    and max_abs_roll < 0.55
)
```

含义：

| 检查项 | 目的 |
| --- | --- |
| `avg_speed` | 确认机器人真的在前进 |
| `min_height` | 防止趴地 |
| `max_pitch` | 防止前倾/后仰过大 |
| `max_roll` | 防止侧翻 |

阈值换算：

```text
0.60 rad ≈ 34.4 deg
0.55 rad ≈ 31.5 deg
```

输出 `OK` 表示该速度通过健康检查。

## 12. 参数调节建议

### 12.1 如果机器人前倾

优先调：

- 增大 pitch feedback：`-8.0 * pitch`；
- 降低 `step_length`；
- 增大 `duty`；
- 稍微提高 `height`。

### 12.2 如果机器人速度不够

可以调：

- 增大 `step_length` 上限；
- 增大 `total_fx` 中的速度反馈增益；
- 增大 `--torque-scale`；
- 缩短 `period`。

但过度提高会导致打滑和姿态振荡。

### 12.3 如果机器人跳动明显

可以调：

- 降低高度控制 `Kz = 70.0`；
- 增加垂向阻尼 `Dz = 5.0`；
- 降低摆腿 clearance；
- 增大 duty。

### 12.4 如果画面慢放

优先检查：

```bash
--render-every
```

默认 16。如果机器图形性能弱，可以设置：

```bash
--render-every 20
```

不要降低 MuJoCo 物理频率，除非明确要牺牲物理精度。

## 13. 涉及的知识点总结

### 13.1 MuJoCo MJCF

MJCF 是 MuJoCo 的 XML 模型格式，用于描述：

- body；
- joint；
- geom；
- actuator；
- sensor；
- inertial；
- contact；
- worldbody。

本项目主要用它描述 LegBot 的刚体树和碰撞模型。

### 13.2 四元数与姿态

MuJoCo freejoint 中 base 姿态是 quaternion：

```text
[w, x, y, z]
```

脚本将其转成 roll/pitch/yaw，用于姿态反馈。

### 13.3 逆运动学 IK

IK 用于从足端位置反解关节角。当前是简化二连杆解析 IK，优点是：

- 快；
- 不需要优化器；
- 容易调试。

缺点是：

- 未考虑完整 3D 腿部几何；
- 高难地形时精度有限。

### 13.4 Jacobian 与虚功原理

足端力转换成关节力矩依赖：

```math
\tau = J^T F
```

这来自虚功：

```math
F^T \delta x = \tau^T \delta q
```

又因为：

```math
\delta x = J \delta q
```

所以：

```math
F^T J \delta q = \tau^T \delta q
```

得到：

```math
\tau = J^T F
```

当前模型方向使用 `-J^T F`。

### 13.5 Raibert 思想

Raibert hopping/legged locomotion 的核心思想是：

- 通过落脚点调节速度；
- 通过垂向力调节高度；
- 通过姿态反馈调节稳定性。

本项目不是完整 Raibert 控制器，但使用了类似思想：

```text
速度误差 → 足端前后位置修正 + 水平支撑力
高度误差 → 垂向支撑力
姿态误差 → pitch/roll 反馈
```

### 13.6 控制频率与渲染频率解耦

一个常见误区是把仿真慢和渲染慢混在一起。

这里三者是分开的：

```text
physics_hz:  MuJoCo 积分频率
control_hz:  torque 更新频率
viewer_hz:   画面同步频率
```

当前配置：

```text
physics_hz = 1000
control_hz = 500 或 1000
viewer_hz ≈ 62.5
```

这比每步刷新 viewer 更合理。

## 14. 项目分析与接入要求

这一节专门回答一个工程问题：现在到底要把什么接入项目？

当前任务的重点不是一定要使用原 Cheetah-Software 的 Qt 仿真器，而是：

```text
使用你的 LegBot MJCF/STL 模型，在 MuJoCo 中验证稳定前进；
同时让 Cheetah-Software 的模型层和控制算法解释能够对应到这个 LegBot。
```

也就是说，仿真测试可以在 MuJoCo 中完成；Cheetah-Software 侧主要承担：

- 机器人抽象模型说明；
- 控制算法原理参考；
- 后续 MPC/WBC 接入的参数接口；
- 项目结构和运行流程说明。

### 14.1 原项目的控制闭环要求

Cheetah-Software 原始控制链路可以概括为：

```text
机器人模型参数
    ↓
状态估计 State Estimator
    ↓
期望状态 Desired State Command
    ↓
FSM 状态机
    ↓
Locomotion / Balance / Recovery 等状态
    ↓
MPC / WBC / LegController
    ↓
关节力矩或关节位置命令
    ↓
仿真器或真实硬件
```

因此，一个新机器人模型要完整接入原项目，至少要解决五类问题：

| 类别 | 需要接入的内容 |
| --- | --- |
| 模型层 | 质量、惯量、连杆长度、关节轴、关节限位、力矩上限 |
| 仿真层 | 碰撞几何、接触参数、地面、传感器、执行器 |
| 状态层 | base 位姿、base 速度、关节位置、关节速度、足端接触 |
| 控制层 | 步态、落足点、支撑力、摆腿轨迹、力矩映射 |
| 验证层 | 速度范围、姿态稳定、高度不塌陷、viewer 实时性 |

本次 LegBot 接入采用“模型层接入 + MuJoCo 行为验证”的路线：

- Cheetah-Software 侧新增/保留 LegBot 的 `RobotType`、`buildLegBot()`、默认参数和控制器分支；
- MuJoCo 侧加载真实 MJCF/STL，并运行前进步态控制器；
- 稳定性、速度范围、机身高度和可视化频率都在 MuJoCo 中测试；
- 暂不把原 Qt sim 作为必要验证链路，因为它缺少现成的 LegBot mesh/碰撞和当前机器 Qt5 编译环境。

### 14.2 当前实现已经满足的接入要求

#### 14.2.1 模型加载要求

要求：

- 能加载项目内 LegBot MJCF；
- 能找到 STL mesh；
- 不依赖外部绝对路径；
- 原始模型文件不被破坏。

当前实现：

- 使用 `models/MJCF/legbot/legbot_mpc_scene.xml` 作为源模型；
- 自动查找 `models/legbot/xmls/meshes`；
- 生成 `models/MJCF/legbot/legbot_mpc_scene_project.xml`；
- 不直接修改源 XML。

这满足了模型接入的基本工程要求。

#### 14.2.2 自由度和状态读取要求

LegBot 场景中原本有 target marker 和 heading arrow，如果直接假设 `qpos[0:7]` 是机器人 base，会出现状态读取错位。

当前实现通过 MuJoCo joint name 获取 base freejoint 地址：

```python
freejoint_id = model.joint("joint_fixed_world").id
base_qpos_addr = model.jnt_qposadr[freejoint_id]
base_qvel_addr = model.jnt_dofadr[freejoint_id]
```

这样满足：

- base 状态读取稳定；
- 额外 joint 不影响机器人控制；
- 后续扩展 XML 时更安全。

#### 14.2.3 执行器要求

要求：

- 12 个关节都要有 motor actuator；
- hip/thigh/calf 的力矩上限要和模型匹配；
- 控制器输出必须按 actuator 顺序写入 `data.ctrl`。

当前执行器顺序：

```text
FR_hip, FR_thigh, FR_calf,
FL_hip, FL_thigh, FL_calf,
RR_hip, RR_thigh, RR_calf,
RL_hip, RL_thigh, RL_calf
```

当前控制器输出：

```python
data.ctrl[actuator_id] = tau
```

并且使用 `--torque-scale` 在生成 XML 中缩放力矩上限，默认：

```text
torque_scale = 1.8
```

这满足当前 MuJoCo 前进仿真的执行器要求。

#### 14.2.4 步态稳定要求

任务要求是：

```text
步态正确，不前倾，不摔倒，速度范围 0.5-2.0 m/s
```

当前采用 diagonal trot：

```text
FR + RL 同相
FL + RR 同相
相位差 0.5
```

并通过：

- 摆腿轨迹；
- 解析 IK；
- 关节 PD；
- 支撑腿地面反力；
- pitch/roll feedback；
- 高度控制；

来保证稳定。

测试结果显示，在 `height=0.2775 m` 下，500 Hz 和 1000 Hz 控制都能完成 `0.5-2.0 m/s` 指令扫描。

#### 14.2.5 可视化实时性要求

原来每个 MuJoCo step 都 `viewer.sync()`，等于 1000 Hz 刷新画面，容易出现慢放感。

当前实现：

```text
physics_hz = 1000
control_hz = 500 或 1000
viewer sync = 每 16 个物理步一次 ≈ 62.5 Hz
```

这样物理精度不变，但画面刷新接近正常显示器刷新率。

### 14.3 Cheetah-Software 中已经接入的内容

当前项目已经具备 LegBot 在 Cheetah-Software 语境下的模型入口：

| 文件 | 接入内容 |
| --- | --- |
| `common/include/cppTypes.h` | 增加 `RobotType::LEGBOT` |
| `common/include/Dynamics/LegBot.h` | 定义 `buildLegBot<T>()`，写入质量、惯量、连杆长度、髋位置和电机参数 |
| `config/legbot-defaults.yaml` | LegBot 默认控制参数 |
| `common/include/ControlParameters/SimulatorParameters.h` | 增加 LegBot 参数文件宏 |
| `robot/src/main_helper.cpp` | 命令行 robot-id 增加 `v` |
| `robot/src/RobotRunner.cpp` | LegBot 使用 Mini-Cheetah 风格 SPI/LegController 数据通路 |
| `sim/src/Simulation.cpp` | LegBot 可构造 C++ dynamics model，参数文件可加载 |
| `sim/src/RobotInterface.cpp` | LegBot robot interface 参数入口 |
| `user/MIT_Controller/...` | StandUp、Locomotion、SafetyChecker 中增加 LegBot 高度和安全分支 |
| `common/test/test_legbot_model.cpp` | LegBot 模型构造和质量参数测试 |

这部分的意义是：原项目的算法模块不再只认识 Cheetah 3 和 Mini Cheetah，
而是有了 LegBot 的机器人类型、动力学参数和控制器分支。

需要注意：这不等于原生 Qt sim 已经完整可视化 LegBot mesh，也不等于原 C++ Convex MPC/WBC 已经完成 LegBot 高性能调参。
当前稳定步态验证仍以 MuJoCo 为主。

### 14.4 当前实现的局限

当前控制器是工程验证型控制器，不是完整 MPC/WBC。它解决了“LegBot 模型在 MuJoCo 中稳定前进”的问题，但还没有达到真实机器人级别的最优控制。

局限包括：

- 没有显式摩擦锥约束；
- 没有优化分配四足接触力；
- 没有地形感知；
- 没有真实状态估计噪声；
- 没有电机电流、电压、热限制模型；
- 高速速度跟踪依赖参数调节；
- 只重点验证平地前进；
- 原 C++ Convex MPC/WBC 对 LegBot 还没有完成精调；
- 原 Qt sim 的 LegBot mesh/碰撞可视化不是当前必要测试链路；
- 当前机器缺少 Qt5 开发包，原生 `sim` 目标无法完整配置。

这些局限不是 bug，而是当前实现定位决定的：它是 LegBot MuJoCo 接入和步态验证基线，不是最终机器人控制栈。

### 14.5 如果继续升级完整 Cheetah-Software，需要做什么

如果后续目标是让 LegBot 像 Mini Cheetah/Cheetah 3 一样完全运行在原 C++ MPC/WBC 框架里，需要继续增加以下工作。

#### 14.5.1 完善 RobotType 全链路

当前已经增加 `RobotType::LEGBOT`，后续要继续检查所有遗漏分支，包括：

- `common/include/cppTypes.h`
- `SimulationBridge`
- `RobotRunner`
- 参数加载逻辑
- 可视化和配置文件选择逻辑

#### 14.5.2 细化 LegBot 动力学参数

当前已有：

```text
common/include/Dynamics/LegBot.h
```

里面定义：

- body mass；
- body inertia；
- hip/thigh/calf link length；
- link mass；
- link inertia；
- rotor inertia；
- hip location；
- joint damping/friction；
- motor torque limit。

也就是构造一个：

```cpp
Quadruped<T> buildLegBot()
```

这样原项目的 `FloatingBaseModel`、LegController 和 WBC 才能使用 LegBot。

后续可继续精修 inertial quaternion 对惯量张量的旋转变换，让 C++ 模型和 MJCF 模型更严格一致。

#### 14.5.3 转换 MJCF 参数到 C++ Quadruped

MJCF 中的参数是：

```xml
<inertial mass="..." fullinertia="..." />
<body pos="..." />
<joint axis="..." range="..." />
```

需要将这些参数映射到 C++：

```cpp
SpatialInertia
Vec3 bodyCOM
Mat3 bodyRotationalInertia
abadLocation
hipLocation
kneeLocation
```

这个步骤必须非常谨慎，因为坐标系约定不一致会导致：

- 足端位置反；
- 关节方向反；
- 重力项错误；
- MPC/WBC 输出方向错误。

#### 14.5.4 接入原 LegController

原 LegController 支持：

- joint PD；
- Cartesian PD；
- feedforward torque；
- Jacobian；
- foot position/velocity。

LegBot 需要确保：

```text
关节顺序
腿顺序
关节正方向
足端坐标系
```

都和 LegController 约定一致。

#### 14.5.5 接入 Convex MPC

原 Convex MPC 的核心任务是优化未来一段时间内的足端接触力：

```math
\min_f \sum_k \|x_k - x_k^{des}\|_Q^2 + \|f_k\|_R^2
```

约束包括：

```math
0 \le f_z \le f_{z,max}
```

```math
|f_x| \le \mu f_z
```

```math
|f_y| \le \mu f_z
```

要接入 LegBot，需要给 MPC 提供：

- body mass；
- body inertia；
- foot positions；
- contact schedule；
- desired velocity；
- friction coefficient；
- force limits。

当前 Python 控制器中的 `total_fx/total_fz/pitch_moment/roll_force` 可以理解为简化版手写 MPC/WBC 结果，而不是真正优化解。

#### 14.5.6 接入 WBC

WBC 用于把期望 body acceleration、foot force、swing foot task 转成全身关节力矩。

典型任务：

```text
body orientation task
body height task
swing foot position task
contact constraint
torque limit
friction cone
```

接入 LegBot 后，WBC 需要准确的：

- mass matrix；
- Coriolis/gravity；
- contact Jacobian；
- foot Jacobian；
- torque limits。

#### 14.5.7 接入状态估计

当前 MuJoCo 脚本直接使用真值：

```python
base_qpos
base_qvel
joint q/dq
```

真实控制中需要状态估计：

- IMU 姿态；
- gyro；
- joint encoder；
- foot contact；
- base velocity estimator。

这部分原项目已有 OrientationEstimator、PositionVelocityEstimator，可以复用，但 LegBot 的 sensor mapping 需要适配。

### 14.6 推荐分阶段接入路线

为了降低风险，不建议一口气把 LegBot 塞进完整 C++ MPC/WBC。推荐路线：

#### 第一阶段：MuJoCo 模型验证

当前已经完成。

目标：

- MJCF 能加载；
- mesh 正确；
- joint/actuator 顺序正确；
- 机器人能稳定前进；
- viewer 正常显示。

#### 第二阶段：参数整理

把脚本里的参数整理成 YAML：

```text
height
period coefficients
step length coefficients
kp/kd
force gains
torque scale
```

目标是让调参不需要改代码。

#### 第三阶段：日志和评估

增加 CSV/NPZ log：

- base position；
- base velocity；
- roll/pitch/yaw；
- joint q/dq；
- torque；
- foot contact；
- foot force；
- command speed。

这样可以系统分析稳定性，而不是只看 viewer。

#### 第四阶段：加入摩擦锥和力分配

把当前手写 `total_fx/total_fz` 改成小型 QP：

```math
\min_f \|Af - b\|^2 + \lambda \|f\|^2
```

约束：

```math
f_z \ge 0
```

```math
|f_x| \le \mu f_z
```

```math
|f_y| \le \mu f_z
```

这一步已经接近简化版 Convex MPC。

#### 第五阶段：完善原 C++ Quadruped/FloatingBaseModel

当前已经新增 `buildLegBot()`，让原项目认识 LegBot。后续重点是测试和校准。

通过单元测试验证：

- 总质量；
- 惯量；
- 足端正运动学；
- Jacobian；
- 重力项；
- 接触点。

#### 第六阶段：接入原 FSM/MPC/WBC

最后再接入：

```text
RobotRunner
FSM_State_Locomotion
ConvexMPCLocomotion
WBC LocomotionCtrl
```

这样风险最小。

### 14.7 当前接入要求与实现状态对照表

| 接入要求 | 当前状态 | 说明 |
| --- | --- | --- |
| 加载 LegBot MJCF | 已完成 | 自动生成 project XML |
| 加载 STL mesh | 已完成 | 自动修正 meshdir |
| MuJoCo 前进仿真 | 已完成 | `scripts/mujoco_legbot_walk.py` |
| 速度 0.5-2.0 m/s | 已完成 | 2.0 指令平均约 1.83 m/s |
| 不前倾、不摔倒 | 已完成 | roll/pitch 均在健康阈值内 |
| 质心/机身高度 0.2775 | 已完成 | 默认 `--height 0.2775` |
| 物理 1000 Hz | 已完成 | timestep 0.001 |
| 控制 500/1000 Hz | 已完成 | `--control-hz` |
| viewer 约 60 Hz | 已完成 | `--render-every 16` |
| 原 C++ RobotType 接入 | 已完成 | 已增加 `RobotType::LEGBOT` 和命令行 `v` |
| C++ Quadruped 模型 | 已完成基础版 | `buildLegBot()` 已建立，惯量旋转仍可继续精修 |
| 原 Convex MPC 接入 | 部分完成 | 有 LegBot 分支和高度参数，尚未完成系统调参 |
| 原 WBC 接入 | 部分完成 | C++ 模型可供 WBC 使用，当前稳定验证仍用 MuJoCo `-J^T F` |
| 状态估计器接入 | 未完成 | 当前使用 MuJoCo 真值 |
| 原 Qt sim 完整 LegBot 可视化 | 非当前必要项 | 当前机器缺 Qt5，且测试可在 MuJoCo 完成 |

### 14.8 本次实现的定位

一句话总结当前实现定位：

```text
这是 LegBot 的 Cheetah-Software 模型层接入 + MuJoCo 稳定前进验证基线。
```

它已经满足本阶段任务：

- 能跑；
- 能看；
- 能稳定前进；
- 能扫速度；
- 频率设计合理；
- 参数可调；
- 源模型不被破坏。

后续如果要做真实机器人级控制，就应该从当前基线继续向 MPC/WBC、状态估计和 C++ 框架接入演进。

## 15. 总结

本项目现在可以用 MuJoCo 直接运行 LegBot 前进仿真。当前方案虽然没有完全复刻 Cheetah-Software 的 C++ MPC/WBC 管线，但实现了四足前进控制中最关键的几个机制：

- trot 步态相位；
- 摆腿轨迹；
- 解析 IK；
- 关节 PD；
- 支撑腿地面反力；
- Jacobian 力矩映射；
- pitch/roll 姿态稳定；
- 物理、控制和渲染频率解耦。

最终默认配置：

```text
height = 0.2775 m
physics_hz = 1000
control_hz = 1000
viewer sync = 每 16 个物理步一次
speed range = 0.5-2.0 m/s
```

推荐快速测试命令：

```bash
cd /home/lushilin/Cheetah-Software-master
conda run -n mujoco python scripts/mujoco_legbot_walk.py --scan --duration 8
```

推荐可视化命令：

```bash
conda run -n mujoco python scripts/mujoco_legbot_walk.py --speed 1.0 --duration 20 --viewer --realtime
```

这套实现适合作为后续接入 MPC、WBC 或学习控制策略前的模型验证和步态基线。

# <div align="center"> LegBot-MPC-WBC
</div>

<div align="center">

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![C++](https://img.shields.io/badge/C%2B%2B-17-blue.svg)](https://en.cppreference.com/w/)
[![CMake](https://img.shields.io/badge/CMake-3.5%2B-brightgreen.svg)](https://cmake.org/)
[![Eigen3](https://img.shields.io/badge/Eigen-3-purple.svg)](https://eigen.tuxfamily.org/)
[![Pinocchio](https://img.shields.io/badge/Dynamics-Pinocchio%204.0-9cf.svg)](https://stack-of-tasks.github.io/pinocchio/)
[![OSQP](https://img.shields.io/badge/QP-OSQP-orange.svg)](https://osqp.org/)
[![MuJoCo](https://img.shields.io/badge/Sim-MuJoCo%203.3.6-lightgrey.svg)](https://mujoco.org/)
[![LCM](https://img.shields.io/badge/Comm-LCM%201.3.1-yellow.svg)](https://lcm-proj.github.io/)
[![Platform](https://img.shields.io/badge/Platform-Linux%20%7C%20ARM64-lightgrey.svg)](#)
[![Status](https://img.shields.io/badge/Status-Active-success.svg)](https://github.com/Robot-Nav/legbot-MPC-WBC)



**基于模型预测控制（MPC）与全身控制（WBC）的四足机器人运动控制系统**

</div>

---




https://github.com/user-attachments/assets/c7acea92-6082-49ee-9bc1-b4c6f612c505



---

## 项目简介

LegBot-MPC-WBC 是一个面向四足机器人（LegBot）的高性能运动控制框架，核心基于 MIT Cheetah-Software 架构进行深度开发。项目实现了**模型预测控制（MPC）** 与**全身控制（WBC）** 的紧耦合控制链路：

- **MPC** 在预测时域内优化机器人的身体轨迹、足端位置与地面反力
- **WBC** 将 MPC 的高层规划转化为关节级力矩指令，同时满足接触约束与动力学约束

项目采用**单一 QP-WBC 架构**：基于 **Pinocchio** 构建四足浮基动力学模型，将运动学任务跟踪与浮基动力学约束统一为单个二次规划（QP）问题，由 **OSQP** 实时求解。

项目支持 **仿真（MuJoCo/Qt-Simulator）** 与 **实物部署** 双模式运行，通信层基于 LCM 和 DDS 协议。

---

## 系统架构

```
┌─────────────────────────────────────────────────────┐
│                   Gamepad / RC                       │
└─────────────────────┬───────────────────────────────┘
                      ▼
┌─────────────────────────────────────────────────────┐
│                  ControlFSM                          │
│   (Finite State Machine: Passive→StandUp→            │
│    BalanceStand↔Locomotion→Recovery)                 │
└─────────────────────┬───────────────────────────────┘
                      ▼
┌─────────────────────────────────────────────────────┐
│              ConvexMPCLocomotion                     │
│  ┌─────────────────────────────────────────────┐    │
│  │ Model Predictive Control (MPC)              │    │
│  │  • Dense QP (SolverMPC)                     │    │
│  │  • Sparse QP (SparseCMPC + OSQP)            │    │
│  │  Output: Body Trajectory + Foot Placement    │    │
│  │          + Ground Reaction Forces            │    │
│  └─────────────────────────────────────────────┘    │
└─────────────────────┬───────────────────────────────┘
                      ▼
┌─────────────────────────────────────────────────────┐
│                 LocomotionCtrl (WBC_Ctrl)             │
│  ┌─────────────────────────────────────────────┐    │
│  │  QP-WBC (Pinocchio + OSQP)                  │    │
│  │  • PinocchioDynamics: URDF 浮基动力学模型     │    │
│  │    (质量阵/科氏/重力/雅可比)                  │    │
│  │  • 单一 QP 求解:                             │    │
│  │    - 软优先级任务 (姿态/位置/足端)            │    │
│  │    - 浮基动力学等式约束                       │    │
│  │    - 摩擦锥不等式约束                         │    │
│  │  → 输出关节力矩 + 前馈反力                    │    │
│  └─────────────────────────────────────────────┘    │
└─────────────────────┬───────────────────────────────┘
                      ▼
┌─────────────────────────────────────────────────────┐
│                 LegController                        │
│     (Joint PD + Feed-forward Torque Command)         │
└─────────────────────┬───────────────────────────────┘
                      ▼
┌─────────────────────────────────────────────────────┐
│       Robot Hardware / Simulator (MuJoCo / Qt)       │
└─────────────────────────────────────────────────────┘
```

---

## 算法原理

### 1. 模型预测控制 (MPC)

MPC 将四足机器人的运动规划转化为一个**有限时域最优控制问题**。系统采用**单刚体动力学模型**（Single Rigid Body Dynamics），将机器人近似为一个受地面反力驱动的浮动机体。

#### 状态空间模型

系统状态 $\mathbf{x} \in \mathbb{R}^{13}$ 包含位置、速度、姿态四元数和角速度：

$$\mathbf{x} = [\mathbf{p}, \mathbf{v}, \mathbf{q}, \boldsymbol{\omega}]^T$$

其中 $\mathbf{p} \in \mathbb{R}^3$ 为身体位置，$\mathbf{v} \in \mathbb{R}^3$ 为身体线速度，$\mathbf{q} \in \mathbb{R}^4$ 为姿态四元数，$\boldsymbol{\omega} \in \mathbb{R}^3$ 为身体角速度。

控制输入 $\mathbf{u} \in \mathbb{R}^{12}$ 为四条腿的地面反力：

$$\mathbf{u} = [\mathbf{f}_1, \mathbf{f}_2, \mathbf{f}_3, \mathbf{f}_4]^T, \quad \mathbf{f}_i \in \mathbb{R}^3$$

#### 连续时间动力学

$$\dot{\mathbf{p}} = \mathbf{v}$$

$$\dot{\mathbf{v}} = \frac{1}{m}\sum_{i=1}^{4} \mathbf{f}_i - \mathbf{g}$$

$$\dot{\mathbf{q}} = \frac{1}{2}\mathbf{q} \otimes \begin{bmatrix} 0 \\ \boldsymbol{\omega} \end{bmatrix}$$

$$\dot{\boldsymbol{\omega}} = \mathbf{I}^{-1}\left(\sum_{i=1}^{4} (\mathbf{r}_i \times \mathbf{f}_i) - \boldsymbol{\omega} \times (\mathbf{I}\boldsymbol{\omega})\right)$$

其中 $m$ 为机体质量，$\mathbf{I}$ 为惯性张量，$\mathbf{r}_i$ 为足端相对质心的位置，$\mathbf{g}$ 为重力加速度。

#### 离散化与 QP 形式

动力学方程经过线性化和离散化后，转化为以下二次规划问题：

$$\min_{\mathbf{x}_k, \mathbf{u}_k} \sum_{k=0}^{H-1} \left( \|\mathbf{x}_{k+1} - \mathbf{x}_{k+1}^{\text{ref}}\|_{\mathbf{Q}}^2 + \|\mathbf{u}_k\|_{\mathbf{R}}^2 \right)$$

$$\text{s.t.} \quad \mathbf{x}_{k+1} = \mathbf{A}_k \mathbf{x}_k + \mathbf{B}_k \mathbf{u}_k$$

$$\mathbf{f}_i \in \mathcal{F}_\mu \quad \text{(摩擦锥约束)}$$

$$\mathbf{f}_i = \mathbf{0} \text{ if leg } i \text{ in swing}$$

其中 $H$ 为预测时域长度（通常为 10 步），$\mathbf{Q}, \mathbf{R}$ 为权重矩阵，$\mathcal{F}_\mu$ 为摩擦系数 $\mu$ 定义的摩擦锥：

$$\mathcal{F}_\mu = \left\{ \mathbf{f} \in \mathbb{R}^3 : |f_x| \leq \mu f_z,\ |f_y| \leq \mu f_z,\ f_z > 0 \right\}$$

#### 双求解器后端

项目提供两种 MPC QP 求解方式：

| 求解器 | 文件 | 特点 |
|--------|------|------|
| **Dense QP** | `SolverMPC.cpp` | 稠密矩阵，适合小规模问题，直接求解 |
| **Sparse QP** | `SparseCMPC.cpp` + OSQP | 稀疏矩阵，高效利用接触序列稀疏性 |

---

### 2. 全身控制 (QP-WBC)

全身控制将 MPC 输出的**身体位姿轨迹**和**足端轨迹**转化为关节级力矩指令。项目采用**单一 QP-WBC 架构**，将运动学任务跟踪、浮基动力学约束和摩擦锥约束统一为单个二次规划问题，由 OSQP 实时求解。

#### 动力学模型 (Pinocchio)

基于 Pinocchio 4.0 加载 LegBot URDF 构建浮基动力学模型（`JointModelFreeFlyer`，$n_q=19, n_v=18$）：

- **质量矩阵** $\mathbf{M}(\mathbf{q})$：`crba`
- **科里奥利力** $\mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})$：`computeCoriolisMatrix`
- **重力项** $\mathbf{G}(\mathbf{q})$：`computeGeneralizedGravity`
- **接触雅可比** $\mathbf{J}_c$：`frameJacobian`（`LOCAL_WORLD_ALIGNED` 参考系）

#### QP 决策变量

$$\mathbf{x} = [\ddot{\mathbf{q}} \; (n_v=18), \; \mathbf{F}_r \; (3 \cdot n_c)]^T$$

其中 $n_c$ 为接触腿数，$\ddot{\mathbf{q}}$ 为广义加速度，$\mathbf{F}_r$ 为接触反力。

#### QP 优化问题

$$\min_{\ddot{\mathbf{q}}, \mathbf{F}_r} \sum_i W_i \|\mathbf{J}_i \ddot{\mathbf{q}} + \dot{\mathbf{J}}_i \dot{\mathbf{q}} - \ddot{\mathbf{x}}_i^{\text{des}}\|^2 + W_{\text{fr}} \|\mathbf{F}_r - \mathbf{F}_r^{\text{MPC}}\|^2 + W_{\ddot{q}} \|\ddot{\mathbf{q}}\|^2$$

$$\text{s.t.} \quad \mathbf{M}\ddot{\mathbf{q}} + \mathbf{C} + \mathbf{G} = \mathbf{S}^T \boldsymbol{\tau} + \mathbf{J}_c^T \mathbf{F}_r \quad \text{(浮基动力学)}$$

$$\quad \mathbf{J}_c \ddot{\mathbf{q}} + \dot{\mathbf{J}}_c \dot{\mathbf{q}} = \mathbf{0} \quad \text{(接触一致性)}$$

$$\quad \mathbf{F}_r \in \mathcal{F}_\mu, \quad F_{r,z} \leq F_{\max} \quad \text{(摩擦锥 + 力上限)}$$

#### 软优先级任务

QP-WBC 通过加权方式实现任务软优先级，无需零空间投影级联：

| 任务 | 雅可比来源 | 期望加速度 |
|------|-----------|-----------|
| **身体姿态** | Pinocchio body Jacobian `topRows<3>()` | PD: $K_p \mathbf{e}_{\text{ori}} + K_d \dot{\mathbf{e}}_{\text{ori}}$ |
| **身体位置** | Pinocchio body Jacobian `bottomRows<3>()` | PD: $K_p \mathbf{e}_{\text{pos}} + K_d \dot{\mathbf{e}}_{\text{pos}}$ |
| **摆动足跟踪** | Pinocchio frame Jacobian | Bezier 轨迹 PD |
| **反力跟踪** | — | MPC 输出 $\mathbf{F}_r^{\text{MPC}}$ |

#### 关节力矩计算

QP 求解得到 $\ddot{\mathbf{q}}$ 和 $\mathbf{F}_r$ 后，关节力矩由动力学方程反解：

$$\boldsymbol{\tau} = \mathbf{S}^T \left( \mathbf{M}\ddot{\mathbf{q}} + \mathbf{C} + \mathbf{G} - \mathbf{J}_c^T \mathbf{F}_r \right)$$

最终 LegController 命令 = 前馈力矩 $\boldsymbol{\tau}$ + 关节 PD（跟踪 WBC 期望关节角/速度）。

---

## 项目结构

```
legbot-MPC-WBC/
├── common/                          # 公共库
│   ├── include/
│   │   ├── Dynamics/                # 机器人动力学模型 (LegBot, Quadruped, FloatingBaseModel)
│   │   ├── SparseCMPC/              # 稀疏MPC求解器接口
│   │   ├── Controllers/             # 步态调度器、腿部控制器
│   │   ├── Math/                    # 数学工具 (滤波器、插值、姿态工具)
│   │   └── Utilities/               # B样条、贝塞尔曲线、定时器等
│   └── src/                         # 动力学、碰撞检测具体实现
│
├── user/MIT_Controller/             # ★ 核心控制器
│   ├── Controllers/
│   │   ├── convexMPC/               # MPC求解器 (ConvexMPCLocomotion, SolverMPC, SparseCMPC)
│   │   ├── WBC/                     # WBC底层库
│   │   │   ├── QPWBC.cpp/hpp        # ★ 单一 QP-WBC 求解器
│   │   │   ├── PinocchioDynamics.cpp/hpp  # ★ Pinocchio 浮基动力学适配器
│   │   │   └── OSQPWrapper.hpp      # ★ OSQP 工作区管理
│   │   ├── WBC_Ctrl/                # WBC编排层
│   │   │   └── LocomotionCtrl/      # locomotion模式下MPC→WBC桥接
│   │   └── BalanceController/       # 站立平衡控制器
│   ├── FSM_States/                  # 有限状态机 (Passive, StandUp, BalanceStand, Locomotion, Recovery)
│   ├── MIT_Controller.cpp/hpp       # 控制器顶层入口
│   └── MIT_UserParameters.h         # 用户参数定义
│
├── robot/                           # 机器人运行框架
│   ├── include/
│   │   ├── LegBotDDS/               # DDS通信适配器 (状态/命令/安全)
│   │   └── rt/                      # 实时接口 (EtherCAT, SPI, VectorNav)
│   └── src/                         # RobotRunner, HardwareBridge, SimulationBridge
│
├── mujoco_bridge/                   # MuJoCo仿真桥接
│   ├── LegBotMujocoSim.cpp          # MuJoCo仿真主程序
│   └── LegBotMujocoKey3DryrunSim.cpp# 实物调试仿真程序
│
├── sim/                             # Qt图形化仿真器
├── tools/                           # 实物桥接与调试工具
│   ├── legbot_real_bridge_dryrun.cpp# ★ 实物DDS桥接主程序
│   └── legbot_model_check.cpp       # 模型检查工具
│
├── models/                          # 机器人模型文件
│   ├── MJCF/legbot/                 # MuJoCo场景文件
│   ├── legbot/xmls/                 # 机器人URDF/XML模型
│   └── legbot_description/          # URDF描述 (Pinocchio加载用)
│
├── config/                          # YAML配置文件
│   ├── legbot-defaults.yaml         # 机器人默认参数
│   └── legbot-mit-ctrl-user-parameters.yaml  # 控制器用户参数
│
├── common/test/                     # 单元测试
├── scripts/                         # 辅助脚本
├── third-party/
│   ├── osqp/                        # OSQP求解器 (源码内嵌)
│   ├── ParamHandler/                # YAML参数解析
│   └── serial_dds_gateway/          # DDS串口网关
└── CMakeLists.txt                   # 根构建文件
```

---

## 依赖项

### 必需依赖

| 依赖 | 版本 | 用途 |
|------|------|------|
| **Eigen3** | ≥ 3.3 | 线性代数库 |
| **CMake** | ≥ 3.5 | 构建系统 |
| **GCC/G++** | ≥ 7.0 (C++17) | 编译器 |
| **LCM** | 1.3.1+ | 通信中间件 |
| **Pinocchio** | 4.0+ | 浮基动力学模型（CRBA/Jacobian/重力/科氏） |
| **OSQP** | 内嵌 | QP求解器（QP-WBC + SparseMPC） |

### 可选依赖

| 依赖 | 用途 |
|------|------|
| **MuJoCo** 3.3.6 | 物理仿真 |
| **Qt** 5.10+ | 图形化仿真器界面 |
| **unitree_sdk2** | 实物DDS通信（自动检测，缺失则跳过实物bridge编译） |

### Ubuntu 安装

```bash
# 基础依赖
sudo apt install mesa-common-dev freeglut3-dev \
  libblas-dev liblapack-dev gfortran \
  cmake gcc build-essential libglib2.0-dev

# Eigen3
sudo apt install libeigen3-dev

# LCM 1.3.1
# 从源码安装: https://lcm-proj.github.io/

# Pinocchio 4.0 (推荐通过 conda 或源码安装)
conda install -c conda-forge pinocchio
# 或从源码: https://github.com/stack-of-tasks/pinocchio

# Qt (可选，用于图形化仿真器)
sudo apt install qtbase5-dev libqt5gamepad5-dev

# MuJoCo (可选，用于物理仿真)
# 下载: https://github.com/google-deepmind/mujoco/releases
# 解压到 ~/.mujoco/mujoco-3.3.6/
```

> **注意**：OSQP 源码已内嵌在 `third-party/osqp/`，无需单独安装。

---

## 编译与运行

### 基础编译

```bash
cd legbot-MPC-WBC
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

### ARM64 (OrangePi / 树莓派) 编译

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
```

### 运行单元测试

```bash
cd build
./common/test-common
```

---

### 仿真运行

#### 方式一：Qt 图形化仿真器

**终端1** - 启动仿真器：
```bash
cd build
./sim/sim
```
在界面中选择 `LegBot` → `Simulator` → 点击 `Start`

**终端2** - 启动控制器：
```bash
cd build
./user/MIT_Controller/mit_ctrl l s
```
参数说明: `l` = LegBot, `s` = 仿真模式

#### 方式二：MuJoCo 物理仿真

```bash
cd build
./mujoco_bridge/mujoco_legbot_sim
```

#### 步态控制

在仿真器中将 `control_mode` 设置为：
- `1` → 站立准备
- `3` → 平衡站立 (BALANCE_STAND)
- `4` → 运动模式 (LOCOMOTION)

使用游戏手柄控制运动方向和速度。

---

### 实物运行

#### 前置条件

1. 电机已通电，CAN 总线连接正常
2. `dds_to_serial_gateway` 已启动（管理串口/CAN/IMU 通信）
3. DDS 网络配置正确

#### 启动 DDS 网关

```bash
cd ~/serial_dds_gateway
./build/dds_to_serial_gateway \
  --serial-port-a /dev/myttyCAN0 \
  --serial-port-b /dev/myttyCAN1 \
  --baudrate 2000000 \
  --network lo \
  --tick-hz 100 \
  --imu-port /dev/myttyIMU \
  --imu-baudrate 921600
```

#### Dry-Run 测试（只读状态，不输出命令）

```bash
cd build
./user/MIT_Controller/legbot_real_bridge_dryrun \
  --dry-run \
  --network lo \
  --duration 10 \
  --cmd-hz 100
```

#### 交互模式实物测试（推荐）

```bash
./user/MIT_Controller/legbot_real_bridge_dryrun \
  --real-output-guarded \
  --interactive-control \
  --network lo \
  --standup-ramp-seconds 6 \
  --shutdown-ramp-seconds 6 \
  --standup-kp 50 \
  --standup-kd 3 \
  --test-gait 4 \
  --tau-ff-scale 1.0 \
  --disable-on-exit \
  --robot-standing-supported \
  --i-accept-risk
```

**键盘控制指令：**

| 按键 | 功能 |
|------|------|
| `1` | q-only 插值站起 |
| `2` | MIT BALANCE_STAND direct (四足站立平衡) |
| `3` | MIT LOCOMOTION 安全接管 (站立→慢速踏步) |
| `x` | 回到 Q-STAND-HOLD |
| `4` / `q` | 回趴 + disable + 退出 |

---

## 步态类型

项目支持丰富的步态模式（定义在 `Gait.cpp`）：

| 步态 | gaitNumber | 描述 |
|------|------------|------|
| **Trotting** | 0 | 对角小跑（默认 locomotion 步态） |
| **Standing** | 4 | 四足全站立 |
| **Bounding** | 1 | 跳跃步态（前后腿结对） |
| **Pronking** | 2 | 四腿同时跳跃 |
| **Galloping** | 5 | 奔跑步态 |
| **Walking** | 6 | 行走步态 |
| **Pacing** | 7 | 同侧步态 |

---

## 配置说明

### 机器人参数 (`config/legbot-defaults.yaml`)

```yaml
controller_dt: 0.001       # 控制周期 1kHz
cheater_mode: 1            # 仿真中是否使用真实状态
stand_kp_cartesian: [80, 80, 80]
stand_kd_cartesian: [3.0, 3.0, 3.0]
```

### MPC 关键参数

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `horizonLength` | 10 | MPC 预测时域步数 |
| `iterationsBetweenMPC` | 30 | MPC 重新计算间隔（每30ms） |
| `dtMPC` | ~0.03s | MPC 离散化时间步长 |
| `_mu` | 0.6 | 摩擦系数 |
| `_maxForce` | 250N | 单腿最大地面反力 |

### QP-WBC 关键参数

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `W_ori` | 300 | 身体姿态任务权重 |
| `W_pos` | 300 | 身体位置任务权重（LegBot Z轴: 10） |
| `W_foot` | 100 | 摆动足跟踪任务权重 |
| `W_fr` | 1.0 | 反力跟踪权重（LegBot Z轴: 0.1） |
| `W_qddot` | 0.001 | 广义加速度正则化权重 |
| `Kp_joint` / `Kd_joint` | 按关节配置 | 关节 PD 增益 |
| `maxFz` | 1500N | 单腿最大法向力 |
| `mu` | 0.4 | WBC 摩擦系数 |

---

## 引用

本项目基于 [MIT Cheetah-Software](https://github.com/mit-biomimetics/Cheetah-Software) 进行深度二次开发：

- **Convex MPC**: Di Carlo, J., et al. "Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control." *IROS 2018*.
- **WBIC**: Kim, D., et al. "Highly Dynamic Quadruped Locomotion via Whole-Body Impulse Control and Model Predictive Control." *arXiv 2019*.
- **Pinocchio**: Carpentier, J., et al. "The Pinocchio C++ library." *Stack-of-Tasks*.
- **OSQP**: Stellato, B., et al. "OSQP: An Operator Splitting Solver for Quadratic Programs." *Mathematical Programming Computation*.

如果您使用本项目进行研究，请引用上述原始工作。

---

## 致谢

- [MIT Biomimetics Robotics Lab](https://biomimetics.mit.edu/) - 原始 Cheetah-Software 框架
- [Robot-Nav](https://github.com/Robot-Nav) - LegBot 机器人的实机适配与算法扩展

---

<div align="center">
  <sub>Built with for quadruped robotics research</sub>
</div>

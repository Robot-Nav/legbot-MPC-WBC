# <div align="center">LegBot-MPC-WBC</div>

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

## 1. 项目简介

LegBot-MPC-WBC 是一个面向四足机器人（LegBot）的高性能运动控制框架，核心基于 [MIT Cheetah-Software](https://github.com/mit-biomimetics/Cheetah-Software) 架构进行深度二次开发。项目实现了 **模型预测控制（MPC）** 与 **全身控制（WBC）** 的紧耦合控制链路，支持 Trot、Bound、Pronk 等多种步态，可在仿真与实物平台上运行。

### 1.1 核心特点

- **单一 QP-WBC 架构**：摒弃传统的 KinWBC + WBIC 两段式层级投影方案，将运动学任务跟踪、浮基动力学等式约束与摩擦锥不等式约束统一为单个二次规划（QP）问题，由 OSQP 实时求解，减少中间误差累积
- **Pinocchio 动力学后端**：基于 Pinocchio 4.0 加载 URDF 构建浮基动力学模型，直接使用 CRBA 质量矩阵、`computeCoriolisMatrix` 科氏力、`computeGeneralizedGravity` 重力项与 `frameJacobian` 接触雅可比，替代原有手写 FloatingBaseModel
- **双 MPC 求解后端**：提供 Dense QP（SolverMPC，稠密直接求解）与 Sparse QP（SparseCMPC + OSQP，利用接触序列稀疏性）两种求解方式
- **完整步态规划链路**：相位式步态调度（OffsetDurationGait）→ Raibert 落脚点启发式 → Bezier 摆动腿轨迹规划
- **仿真与实物双模式**：支持 Qt 图形化仿真器、MuJoCo 物理仿真、DDS 实物部署三种运行环境
- **C++17 标准**：Pinocchio 4.0 强制要求 C++17，项目已全面升级

### 1.2 控制链路概览

```
用户输入 (Gamepad/RC)
    │
    ▼
ControlFSM (有限状态机: Passive → StandUp → BalanceStand ↔ Locomotion → Recovery)
    │
    ▼
ConvexMPCLocomotion (MPC层)
    │  ├─ 单刚体动力学模型预测
    │  ├─ Dense QP / Sparse QP 求解
    │  └─ 输出: 身体轨迹 + 足端位置 + 地面反力
    │
    ▼
LocomotionCtrl → QP-WBC (WBC层)
    │  ├─ Pinocchio 浮基动力学 (质量阵/科氏/重力/雅可比)
    │  ├─ 软优先级任务 (姿态/位置/摆动足/反力跟踪)
    │  ├─ 浮基动力学等式 + 接触一致性等式
    │  ├─ 摩擦锥不等式 + 力上限
    │  └─ 输出: 广义加速度 q̈ + 接触反力 Fr → 关节力矩 τ
    │
    ▼
LegController (关节 PD + 前馈力矩)
    │
    ▼
Robot Hardware / Simulator (MuJoCo / Qt / DDS)
```

---

## 2. 系统架构

### 2.1 整体架构图

```
┌──────────────────────────────────────────────────────────┐
│                     Gamepad / RC                          │
└────────────────────────┬─────────────────────────────────┘
                         ▼
┌──────────────────────────────────────────────────────────┐
│                      ControlFSM                           │
│   有限状态机:                                              │
│   Passive → StandUp → BalanceStand ↔ Locomotion          │
│                     ↘ RecoveryStand ↗                     │
│   状态转换由 control_mode / RC_mode / 安全检查触发          │
└────────────────────────┬─────────────────────────────────┘
                         ▼
┌──────────────────────────────────────────────────────────┐
│                 ConvexMPCLocomotion                        │
│  ┌────────────────────────────────────────────────────┐  │
│  │              Model Predictive Control              │  │
│  │                                                    │  │
│  │  • 单刚体动力学模型 (SRBD)                          │  │
│  │  • 预测时域 H=10, 离散步长 dt≈0.03s                 │  │
│  │  • Dense QP  (SolverMPC)                           │  │
│  │  • Sparse QP (SparseCMPC + OSQP)                   │  │
│  │                                                    │  │
│  │  输出:                                              │  │
│  │    - 期望身体位姿/速度/加速度                        │  │
│  │    - 期望足端位置/速度/加速度 (Bezier轨迹)           │  │
│  │    - 期望地面反力 Fr_des                            │  │
│  │    - 接触状态 contact_state                         │  │
│  └────────────────────────────────────────────────────┘  │
└────────────────────────┬─────────────────────────────────┘
                         ▼
┌──────────────────────────────────────────────────────────┐
│               LocomotionCtrl (WBC_Ctrl)                    │
│  ┌────────────────────────────────────────────────────┐  │
│  │              QP-WBC (Pinocchio + OSQP)             │  │
│  │                                                    │  │
│  │  PinocchioDynamics:                                │  │
│  │    - URDF 浮基模型 (FreeFlyer, nq=19, nv=18)       │  │
│  │    - crba → 质量矩阵 M(q)                          │  │
│  │    - computeCoriolisMatrix → C(q, q̇)              │  │
│  │    - computeGeneralizedGravity → G(q)             │  │
│  │    - frameJacobian → Jc (LOCAL_WORLD_ALIGNED)     │  │
│  │                                                    │  │
│  │  单一 QP 求解:                                      │  │
│  │    决策变量: x = [q̈; Fr]                           │  │
│  │    代价: 软优先级任务 + 反力跟踪 + 正则化             │  │
│  │    等式: 浮基动力学(6) + 接触一致性(3·nc)           │  │
│  │    不等式: 摩擦锥(6·nc) + 法向力上限                │  │
│  │                                                    │  │
│  │  → 输出: 关节力矩 τ + 前馈反力 Fr                   │  │
│  └────────────────────────────────────────────────────┘  │
└────────────────────────┬─────────────────────────────────┘
                         ▼
┌──────────────────────────────────────────────────────────┐
│                      LegController                         │
│    关节 PD 跟踪 (kp/kd) + 前馈力矩 τ_ff                    │
│    摆动腿: 笛卡尔空间阻抗控制 (kpCartesian/kdCartesian)    │
│    支撑腿: 力矩前馈 + 关节 PD                              │
└────────────────────────┬─────────────────────────────────┘
                         ▼
┌──────────────────────────────────────────────────────────┐
│            Robot Hardware / Simulator                      │
│    MuJoCo 3.3.6 / Qt Simulator / DDS 实物                  │
└──────────────────────────────────────────────────────────┘
```

### 2.2 数据流说明

| 阶段 | 输入 | 输出 | 频率 |
|------|------|------|------|
| **MPC** | 当前状态 (位置/速度/姿态/角速度) + 步态调度 + 用户指令 | 期望身体轨迹 + 足端轨迹 + 地面反力 | ~33 Hz (每 30ms) |
| **QP-WBC** | MPC 输出 + Pinocchio 动力学 + 当前关节状态 | 关节力矩 + 前馈反力 | 1000 Hz (controller_dt) |
| **LegController** | WBC 力矩/位姿命令 + 关节反馈 | 电机驱动指令 | 1000 Hz |

### 2.3 有限状态机 (FSM)

| 状态 | 名称 | control_mode | 说明 |
|------|------|-------------|------|
| 0 | PASSIVE | K_PASSIVE | 被动模式，电机不输出力矩 |
| 1 | JOINT_PD | K_JOINT_PD | 关节空间 PD 控制 |
| 2 | IMPEDANCE_CONTROL | K_IMPEDANCE_CONTROL | 阻抗控制 |
| 3 | BALANCE_STAND | K_BALANCE_STAND | 四足站立平衡 (QP-WBC) |
| 4 | LOCOMOTION | K_LOCOMOTION | 运动模式 (MPC + QP-WBC) |
| 5 | STAND_UP | K_STAND_UP | 站起插值 |
| 6 | RECOVERY_STAND | K_RECOVERY_STAND | 跌倒恢复 |

---

## 3. 算法原理

### 3.1 模型预测控制 (MPC)

MPC 将四足机器人的运动规划转化为一个 **有限时域最优控制问题**。系统采用 **单刚体动力学模型**（Single Rigid Body Dynamics, SRBD），将机器人近似为一个受地面反力驱动的浮动机体。

#### 3.1.1 状态空间模型

系统状态 $\mathbf{x} \in \mathbb{R}^{13}$ 包含位置、速度、姿态四元数和角速度：

$$\mathbf{x} = [\mathbf{p}, \mathbf{v}, \mathbf{q}, \boldsymbol{\omega}]^T$$

其中：
- $\mathbf{p} \in \mathbb{R}^3$ — 身体位置（世界系）
- $\mathbf{v} \in \mathbb{R}^3$ — 身体线速度（世界系）
- $\mathbf{q} \in \mathbb{R}^4$ — 姿态四元数（世界系，MIT 约定 $[w, x, y, z]$）
- $\boldsymbol{\omega} \in \mathbb{R}^3$ — 身体角速度（体系）

控制输入 $\mathbf{u} \in \mathbb{R}^{12}$ 为四条腿的地面反力：

$$\mathbf{u} = [\mathbf{f}_1, \mathbf{f}_2, \mathbf{f}_3, \mathbf{f}_4]^T, \quad \mathbf{f}_i \in \mathbb{R}^3$$

#### 3.1.2 连续时间动力学

$$\dot{\mathbf{p}} = \mathbf{v}$$

$$\dot{\mathbf{v}} = \frac{1}{m}\sum_{i=1}^{4} \mathbf{f}_i - \mathbf{g}$$

$$\dot{\mathbf{q}} = \frac{1}{2}\mathbf{q} \otimes \begin{bmatrix} 0 \\ \boldsymbol{\omega} \end{bmatrix}$$

$$\dot{\boldsymbol{\omega}} = \mathbf{I}^{-1}\left(\sum_{i=1}^{4} (\mathbf{r}_i \times \mathbf{f}_i) - \boldsymbol{\omega} \times (\mathbf{I}\boldsymbol{\omega})\right)$$

其中：
- $m$ — 机体质量
- $\mathbf{I} \in \mathbb{R}^{3\times3}$ — 体系惯性张量
- $\mathbf{r}_i \in \mathbb{R}^3$ — 第 $i$ 条腿足端相对质心的位置向量
- $\mathbf{g} \in \mathbb{R}^3$ — 重力加速度向量

#### 3.1.3 离散化与 QP 形式

将连续动力学在当前状态处线性化，并按 MPC 离散步长 $\Delta t$ 离散化，得到离散状态空间：

$$\mathbf{x}_{k+1} = \mathbf{A}_k \mathbf{x}_k + \mathbf{B}_k \mathbf{u}_k$$

在预测时域 $H$ 内，MPC 求解以下二次规划问题：

$$\min_{\mathbf{x}_k, \mathbf{u}_k} \sum_{k=0}^{H-1} \left( \|\mathbf{x}_{k+1} - \mathbf{x}_{k+1}^{\text{ref}}\|_{\mathbf{Q}}^2 + \|\mathbf{u}_k\|_{\mathbf{R}}^2 + \|\Delta \mathbf{u}_k\|_{\mathbf{R}_\Delta}^2 \right)$$

$$\text{s.t.} \quad \mathbf{x}_{k+1} = \mathbf{A}_k \mathbf{x}_k + \mathbf{B}_k \mathbf{u}_k$$

$$\mathbf{f}_{i,k} = \mathbf{0} \quad \text{if leg } i \text{ in swing at step } k$$

$$\mathbf{f}_{i,k} \in \mathcal{F}_\mu \quad \text{(摩擦锥约束)}$$

其中：
- $H = 10$ — 预测时域步数
- $\mathbf{Q}, \mathbf{R}, \mathbf{R}_\Delta$ — 状态跟踪、控制量、控制量变化率的权重矩阵
- $\mathbf{x}^{\text{ref}}$ — 参考轨迹（由用户指令和步态规划生成）

摩擦锥 $\mathcal{F}_\mu$ 定义为：

$$\mathcal{F}_\mu = \left\{ \mathbf{f} \in \mathbb{R}^3 : |f_x| \leq \mu f_z,\ |f_y| \leq \mu f_z,\ 0 < f_z \leq F_{\max} \right\}$$

#### 3.1.4 双求解器后端

项目提供两种 MPC QP 求解方式，可通过编译选项切换：

| 求解器 | 源文件 | 特点 | 适用场景 |
|--------|--------|------|----------|
| **Dense QP** | `SolverMPC.cpp` | 稠密矩阵消元，将状态变量消去后仅对控制变量求解 | 计算量稳定，适合 ARM64 嵌入式平台 |
| **Sparse QP** | `SparseCMPC.cpp` + OSQP | 保留完整稀疏结构，利用接触序列的稀疏性 | 精度更高，适合 PC 端仿真 |

### 3.2 全身控制 (QP-WBC)

全身控制将 MPC 输出的 **身体位姿轨迹** 和 **足端轨迹** 转化为关节级力矩指令。项目采用 **单一 QP-WBC 架构**，将运动学任务跟踪、浮基动力学约束和摩擦锥约束统一为单个二次规划问题，由 OSQP 实时求解。

#### 3.2.1 动力学模型 (Pinocchio)

基于 Pinocchio 4.0 加载 LegBot URDF 构建浮基动力学模型：

| 项目 | 值 | 说明 |
|------|-----|------|
| 关节模型 | `JointModelFreeFlyer` | 6-DOF 浮基（3 平移 + 3 旋转） |
| 广义坐标维度 | $n_q = 19$ | 浮基 7 (四元数+位置) + 12 关节 |
| 广义速度维度 | $n_v = 18$ | 浮基 6 + 12 关节 |

Pinocchio 提供的动力学计算接口：

| 动力学量 | Pinocchio 接口 | 数学符号 |
|----------|---------------|---------|
| 质量矩阵 | `crba(model, data, q)` | $\mathbf{M}(\mathbf{q}) \in \mathbb{R}^{18\times18}$ |
| 科氏力矩阵 | `computeCoriolisMatrix(model, data, q, v)` | $\mathbf{C}(\mathbf{q}, \dot{\mathbf{q}}) \in \mathbb{R}^{18\times18}$ |
| 重力项 | `computeGeneralizedGravity(model, data, q)` | $\mathbf{G}(\mathbf{q}) \in \mathbb{R}^{18}$ |
| 接触雅可比 | `frameJacobian(model, data, q, frameId, LOCAL_WORLD_ALIGNED)` | $\mathbf{J}_c \in \mathbb{R}^{3n_c \times 18}$ |
| 雅可比时间导数 | `getFrameJacobianTimeVariation(...)` | $\dot{\mathbf{J}}_c$ |
| 正运动学 | `framesForwardKinematics(model, data, q)` | 更新所有帧位姿 |

> **约定转换**：四元数 MIT $[w, x, y, z]$ ↔ Pinocchio $[x, y, z, w]$；浮基速度 MIT $[\boldsymbol{\omega}, \mathbf{v}]$ ↔ Pinocchio $[\mathbf{v}, \boldsymbol{\omega}]$。这些转换在 `PinocchioDynamics` 适配器中处理。

#### 3.2.2 QP 决策变量

$$\mathbf{x} = \begin{bmatrix} \ddot{\mathbf{q}} \\ \mathbf{F}_r \end{bmatrix} \in \mathbb{R}^{n_v + 3n_c}$$

其中：
- $\ddot{\mathbf{q}} \in \mathbb{R}^{18}$ — 广义加速度（浮基 6 + 关节 12）
- $\mathbf{F}_r \in \mathbb{R}^{3n_c}$ — 接触反力（$n_c$ 为接触腿数，Trot 步态下 $n_c = 2$）

#### 3.2.3 QP 优化问题

**代价函数**（软优先级任务 + 反力跟踪 + 正则化）：

$$\min_{\ddot{\mathbf{q}}, \mathbf{F}_r} \quad \underbrace{\sum_i W_i \|\mathbf{J}_i \ddot{\mathbf{q}} + \dot{\mathbf{J}}_i \dot{\mathbf{q}} - \ddot{\mathbf{x}}_i^{\text{des}}\|^2}_{\text{运动学任务跟踪}} + \underbrace{W_{\text{fr}} \|\mathbf{F}_r - \mathbf{F}_r^{\text{MPC}}\|^2}_{\text{反力跟踪}} + \underbrace{W_{\ddot{q}} \|\ddot{\mathbf{q}}\|^2}_{\text{正则化}}$$

**等式约束**：

$$\mathbf{M}\ddot{\mathbf{q}} + \mathbf{C}\dot{\mathbf{q}} + \mathbf{G} = \mathbf{S}^T \boldsymbol{\tau} + \mathbf{J}_c^T \mathbf{F}_r \quad \text{(浮基动力学方程, 6 行)}$$

$$\mathbf{J}_c \ddot{\mathbf{q}} + \dot{\mathbf{J}}_c \dot{\mathbf{q}} = \mathbf{0} \quad \text{(接触一致性, } 3n_c \text{ 行)}$$

其中 $\mathbf{S} = [\mathbf{0}_{12\times6}, \mathbf{I}_{12}]$ 为关节选择矩阵（浮基 6 个 DOF 无驱动）。

**不等式约束**（摩擦锥 + 法向力上限，每条接触腿 6 行）：

$$-\mu F_{r,z} \leq F_{r,x} \leq \mu F_{r,z}$$

$$-\mu F_{r,z} \leq F_{r,y} \leq \mu F_{r,z}$$

$$0 \leq F_{r,z} \leq F_{\max}$$

#### 3.2.4 软优先级任务

QP-WBC 通过加权方式实现任务软优先级，无需零空间投影级联。各任务及其雅可比来源：

| 任务 | 优先级权重 | 雅可比来源 | 期望加速度计算 |
|------|-----------|-----------|--------------|
| **身体姿态** | $W_{\text{ori}} = 300$ | Pinocchio body Jacobian `topRows<3>()` | PD: $K_p^{\text{ori}} \mathbf{e}_{\text{ori}} + K_d^{\text{ori}} \dot{\mathbf{e}}_{\text{ori}}$ |
| **身体位置** | $W_{\text{pos}} = 300$ | Pinocchio body Jacobian `bottomRows<3>()` | PD: $K_p^{\text{pos}} \mathbf{e}_{\text{pos}} + K_d^{\text{pos}} \dot{\mathbf{e}}_{\text{pos}}$ |
| **摆动足跟踪** | $W_{\text{foot}} = 100$ | Pinocchio frame Jacobian | Bezier 轨迹 PD |
| **反力跟踪** | $W_{\text{fr}} = 1.0$ | — | MPC 输出 $\mathbf{F}_r^{\text{MPC}}$ |
| **正则化** | $W_{\ddot{q}} = 0.001$ | — | $\ddot{\mathbf{q}} \to \mathbf{0}$ |

> **与 MIT Task 的等价关系**：Pinocchio body Jacobian（`LOCAL_WORLD_ALIGNED` 参考系）的 `topRows<3>()` 等价于 MIT `BodyOriTask` 的 $[\mathbf{R}_{wb}, \mathbf{0}]$；`bottomRows<3>()` 等价于 MIT `BodyPosTask` 的 $[\mathbf{0}, \mathbf{R}_{wb}, \mathbf{0}]$。

> **姿态误差计算**：$\mathbf{e}_{\text{ori}} = \text{log3}(\mathbf{R}_{wb}^{\text{des}} \cdot \mathbf{R}_{body}^T)$，其中 `log3` 为 SO(3) 对数映射。

#### 3.2.5 关节力矩计算

QP 求解得到 $\ddot{\mathbf{q}}$ 和 $\mathbf{F}_r$ 后，关节力矩由动力学方程反解：

$$\boldsymbol{\tau} = \mathbf{S}^T \left( \mathbf{M}\ddot{\mathbf{q}} + \mathbf{C}\dot{\mathbf{q}} + \mathbf{G} - \mathbf{J}_c^T \mathbf{F}_r \right)$$

最终 LegController 命令：

- **支撑腿**：前馈力矩 $\boldsymbol{\tau}$ + 关节 PD（跟踪 WBC 期望关节角/速度）
- **摆动腿**：笛卡尔空间阻抗控制（跟踪 Bezier 摆动轨迹），`kpCartesian` / `kdCartesian`

#### 3.2.6 OSQP 求解策略

QP-WBC 使用 OSQP 求解器，采用以下优化策略保证实时性：

| 策略 | 说明 |
|------|------|
| **稀疏结构复用** | `osqp_setup` 仅在首次调用时建立 CSC 稀疏结构；后续每拍用 `osqp_update_P_A` / `osqp_update_lin_cost` / `osqp_update_bounds` 更新数值，避免重建 |
| **Warm Start** | 用上一拍解作为初始猜测，调用 `osqp_warm_start_x`，加速收敛 |
| **绝对容差** | `eps_abs = 1e-3`，`eps_rel = 0`，适应实时控制需求 |

### 3.3 步态规划

#### 3.3.1 相位式步态调度 (OffsetDurationGait)

步态调度器基于相位偏移和占空比描述每条腿的接触/摆动状态：

- **占空比 (duty)**：一条腿在步态周期中处于支撑相的比例
- **相位偏移 (offset)**：各腿步态相位的错位，实现不同步态模式
- **步态周期**：一个完整步态循环的时间

| 步态 | 前左(FL) | 前右(FR) | 后左(HL) | 后右(HR) | 占空比 |
|------|---------|---------|---------|---------|--------|
| **Trot** | 0.0 | 0.5 | 0.5 | 0.0 | 0.5 |
| **Bound** | 0.0 | 0.0 | 0.5 | 0.5 | 0.5 |
| **Pronk** | 0.0 | 0.0 | 0.0 | 0.0 | 0.4 |
| **Walk** | 0.0 | 0.5 | 0.25 | 0.75 | 0.75 |

#### 3.3.2 Raibert 落脚点规划

摆动腿的落足点由 Raibert 启发式确定：

$$\mathbf{p}_{\text{foot}}^{\text{des}} = \mathbf{p}_{\text{hip}} + \frac{\mathbf{v}_{\text{cmd}} \cdot T_{\text{stance}}}{2} + k_{\text{Raibert}} (\mathbf{v}_{\text{cmd}} - \mathbf{v}_{\text{body}})$$

其中 $T_{\text{stance}}$ 为支撑相时长，$k_{\text{Raibert}}$ 为反馈增益。

#### 3.3.3 Bezier 摆动腿轨迹

摆动腿轨迹使用半周期 Bezier 曲线规划，保证起止速度为零、中点达到期望抬腿高度：

$$\mathbf{p}_{\text{swing}}(s) = \sum_{i=0}^{n} B_i^n(s) \cdot \mathbf{P}_i, \quad s \in [0, 1]$$

其中 $s$ 为归一化摆动相位，$\mathbf{P}_i$ 为控制点（起点、中点高抬、终点），$B_i^n$ 为 Bernstein 基函数。

---

## 4. 项目结构

```
legbot-MPC-WBC/
├── common/                              # 公共库
│   ├── include/
│   │   ├── Dynamics/                    # 机器人动力学模型
│   │   │   ├── LegBot.h                 #   LegBot 机器人定义
│   │   │   ├── Quadruped.h              #   四足机器人抽象接口
│   │   │   └── FloatingBaseModel.h      #   浮基模型（仿真碰撞用）
│   │   ├── SparseCMPC/                  # 稀疏 MPC 求解器接口
│   │   ├── Controllers/                 # 步态调度器、腿部控制器
│   │   │   ├── GaitScheduler.h          #   相位式步态调度
│   │   │   ├── LegController.h          #   腿部控制接口
│   │   │   └── DesiredStateCommand.h    #   用户指令解析
│   │   ├── Math/                        # 数学工具
│   │   │   ├── Filters.h                #   一阶/二阶低通滤波
│   │   │   ├── Interpolation.h          #   线性/三次插值
│   │   │   └── OrientationTools.h       #   四元数/旋转矩阵工具
│   │   └── Utilities/                   # B样条、Bezier曲线、定时器
│   └── src/                             # 动力学、碰撞检测具体实现
│
├── user/MIT_Controller/                 # ★ 核心控制器
│   ├── Controllers/
│   │   ├── convexMPC/                   # MPC 求解器
│   │   │   ├── ConvexMPCLocomotion.cpp  #   MPC 主逻辑（步态+规划+QP）
│   │   │   ├── SolverMPC.cpp            #   Dense QP 求解器
│   │   │   ├── SparseCMPC.cpp           #   Sparse QP 求解器（OSQP）
│   │   │   ├── Gait.cpp                 #   步态定义（Trot/Bound/Pronk...）
│   │   │   └── FootSwingTrajectory.cpp  #   Bezier 摆动腿轨迹
│   │   │
│   │   ├── WBC/                         # ★ WBC 底层库
│   │   │   ├── QPWBC.cpp / .hpp         #   ★ 单一 QP-WBC 求解器
│   │   │   ├── PinocchioDynamics.cpp / .hpp  # ★ Pinocchio 浮基动力学适配器
│   │   │   └── OSQPWrapper.hpp          #   ★ OSQP 工作区管理（稀疏复用+warm start）
│   │   │
│   │   ├── WBC_Ctrl/                    # WBC 编排层
│   │   │   ├── WBC_Ctrl.cpp / .hpp      #   WBC 控制器基类
│   │   │   └── LocomotionCtrl/          #   Locomotion 模式下 MPC→WBC 桥接
│   │   │       ├── LocomotionCtrl.cpp   #     任务设定+参数配置+力矩输出
│   │   │       └── LocomotionCtrl.hpp
│   │   │
│   │   └── BalanceController/           # 站立平衡控制器（FSM 辅助）
│   │
│   ├── FSM_States/                      # 有限状态机
│   │   ├── ControlFSM.cpp / .h          #   FSM 主控（状态注册+转换调度）
│   │   ├── FSM_State.cpp / .h           #   状态基类
│   │   ├── FSM_State_Passive.cpp        #   被动模式
│   │   ├── FSM_State_StandUp.cpp        #   站起插值
│   │   ├── FSM_State_BalanceStand.cpp   #   平衡站立（QP-WBC 四足着地）
│   │   ├── FSM_State_Locomotion.cpp     #   运动模式（MPC + QP-WBC）
│   │   ├── FSM_State_RecoveryStand.cpp  #   跌倒恢复
│   │   └── SafetyChecker.cpp            #   安全检查（姿态/足端/力）
│   │
│   ├── MIT_Controller.cpp / .hpp        # 控制器顶层入口
│   ├── MIT_UserParameters.h             # 用户参数定义（增益/权重/步态）
│   └── main.cpp                         # 主函数
│
├── robot/                               # 机器人运行框架
│   ├── include/
│   │   ├── LegBotDDS/                   # DDS 通信适配器
│   │   │   ├── LegBotDDSBridge.h        #   DDS 状态/命令/安全桥接
│   │   │   └── LegBotDDSTypes.h         #   DDS 消息类型定义
│   │   └── rt/                          # 实时接口
│   │       ├── rt_rc_interface.h        #   RC/手柄接口
│   │       └── rt_interface_lcm.h       #   LCM 接口
│   └── src/
│       ├── RobotRunner.cpp              #   机器人控制主循环
│       ├── HardwareBridge.cpp           #   硬件桥接（DDS）
│       └── SimulationBridge.cpp         #   仿真桥接（LCM）
│
├── mujoco_bridge/                       # MuJoCo 仿真桥接
│   ├── LegBotMujocoSim.cpp              #   MuJoCo 仿真主程序
│   └── LegBotMujocoKey3DryrunSim.cpp    #   实物调试仿真程序
│
├── sim/                                 # Qt 图形化仿真器
│   ├── main.cpp                         #   仿真器入口
│   ├── Simulator.cpp                    #   仿真器核心
│   └── Graphics3D.cpp                   #   3D 渲染
│
├── tools/                               # 实物桥接与调试工具
│   ├── legbot_real_bridge_dryrun.cpp    # ★ 实物 DDS 桥接主程序（交互模式）
│   ├── legbot_model_check.cpp           # 模型检查工具
│   ├── legbot_real_bridge_gait0_minimal.cpp  # gait0 最小测试
│   └── analyze_legbot_bridge_csv.py     # CSV 日志分析脚本
│
├── models/                              # 机器人模型文件
│   ├── MJCF/legbot/                     # MuJoCo 场景文件 (.xml)
│   ├── legbot/xmls/                     # 机器人 URDF/XML 模型 + 网格
│   └── legbot_description/              # URDF 描述（Pinocchio 加载用）
│       ├── urdf/legbot_description.urdf
│       └── meshes/                      #   STL 网格文件
│
├── config/                              # YAML 配置文件
│   ├── legbot-defaults.yaml             # 机器人默认参数（dt/增益/安全限制）
│   └── legbot-mit-ctrl-user-parameters.yaml  # 控制器用户参数
│
├── common/test/                         # 单元测试
├── scripts/                             # 辅助脚本
├── third-party/
│   ├── osqp/                            # OSQP 求解器（源码内嵌）
│   ├── qpOASES/                         # qpOASES 求解器（MPC 备选）
│   ├── ParamHandler/                    # YAML 参数解析库
│   ├── serial_dds_gateway/              # DDS 串口/CAN 网关
│   └── vectornav/                       # VectorNav IMU 驱动
├── lcm-types/                           # LCM 消息类型定义
├── CMakeLists.txt                       # 根构建文件
├── .gitignore
└── LICENSE
```

---

## 5. 依赖项

### 5.1 必需依赖

| 依赖 | 版本要求 | 用途 | 安装方式 |
|------|---------|------|---------|
| **CMake** | ≥ 3.5 | 构建系统 | `apt install cmake` |
| **GCC / G++** | ≥ 7.0 (C++17) | 编译器 | `apt install build-essential` |
| **Eigen3** | ≥ 3.3 | 线性代数库 | `apt install libeigen3-dev` |
| **LCM** | 1.3.1+ | 通信中间件（仿真/LCM 模式） | 源码安装 |
| **Pinocchio** | 4.0+ | 浮基动力学模型（CRBA/Jacobian/重力/科氏） | conda 或源码安装 |
| **OSQP** | 内嵌 | QP 求解器（QP-WBC + SparseMPC） | 已在 `third-party/osqp/` |

### 5.2 可选依赖

| 依赖 | 版本 | 用途 | 缺失时影响 |
|------|------|------|-----------|
| **MuJoCo** | 3.3.6 | 物理仿真 | 跳过 MuJoCo bridge 编译 |
| **Qt** | 5.10+ | 图形化仿真器界面 | 跳过 sim 编译 |
| **unitree_sdk2** | — | 实物 DDS 通信 | 跳过实物 bridge 编译 |
| **BLAS / LAPACK** | — | 矩阵运算后端 | Pinocchio 性能下降 |

### 5.3 Ubuntu 安装指南

```bash
# ========== 基础依赖 ==========
sudo apt update
sudo apt install -y cmake build-essential gcc g++ \
  mesa-common-dev freeglut3-dev \
  libblas-dev liblapack-dev gfortran \
  libglib2.0-dev libeigen3-dev

# ========== LCM 1.3.1 ==========
# 从源码安装: https://lcm-proj.github.io/
git clone https://github.com/lcm-proj/lcm.git
cd lcm && mkdir build && cd build
cmake .. && make -j$(nproc) && sudo make install
sudo ldconfig

# ========== Pinocchio 4.0 ==========
# 方式一: conda (推荐)
conda install -c conda-forge pinocchio

# 方式二: 源码编译
git clone --recursive https://github.com/stack-of-tasks/pinocchio.git
cd pinocchio && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc) && sudo make install
sudo ldconfig

# ========== Qt 5 (可选，图形化仿真器) ==========
sudo apt install -y qtbase5-dev libqt5gamepad5-dev

# ========== MuJoCo 3.3.6 (可选，物理仿真) ==========
# 下载: https://github.com/google-deepmind/mujoco/releases
# 解压到 ~/.mujoco/mujoco-3.3.6/
# 设置环境变量:
export MUJOCO_ROOT=~/.mujoco/mujoco-3.3.6
```

> **注意**：OSQP 源码已内嵌在 `third-party/osqp/`，无需单独安装。

---

## 6. 编译与运行

### 6.1 基础编译

```bash
cd legbot-MPC-WBC
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

编译产物：
- `user/MIT_Controller/mit_ctrl` — 控制器主程序
- `sim/sim` — Qt 图形化仿真器
- `mujoco_bridge/mujoco_legbot_sim` — MuJoCo 仿真
- `user/MIT_Controller/legbot_real_bridge_dryrun` — 实物 DDS 桥接
- `common/test-common` — 单元测试

### 6.2 ARM64 (OrangePi / 树莓派) 编译

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
```

> ARM64 平台建议使用 Dense QP (SolverMPC) 而非 Sparse QP，计算量更稳定。

### 6.3 运行单元测试

```bash
cd build
./common/test-common
```

### 6.4 仿真运行

#### 方式一：Qt 图形化仿真器

**终端 1** — 启动仿真器：

```bash
cd build
./sim/sim
```

在界面中选择 `LegBot` → `Simulator` → 点击 `Start`

**终端 2** — 启动控制器：

```bash
cd build
./user/MIT_Controller/mit_ctrl l s
```

参数说明：`l` = LegBot 机器人，`s` = 仿真模式

#### 方式二：MuJoCo 物理仿真

```bash
cd build
./mujoco_bridge/mujoco_legbot_sim
```

#### 步态控制

在仿真器界面或 LCM 控制面板中将 `control_mode` 设置为：

| control_mode | 状态 | 说明 |
|-------------|------|------|
| `1` | STAND_UP | 站立准备（关节插值站起） |
| `3` | BALANCE_STAND | 平衡站立（QP-WBC 四足着地） |
| `4` | LOCOMOTION | 运动模式（MPC + QP-WBC + 步态规划） |
| `6` | RECOVERY_STAND | 跌倒恢复 |

使用游戏手柄控制运动方向和速度。

### 6.5 实物运行

#### 前置条件

1. 电机已通电，CAN 总线连接正常
2. `dds_to_serial_gateway` 已启动（管理串口/CAN/IMU 通信）
3. DDS 网络配置正确（同一网段或 `--network lo`）

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

| 按键 | 功能 | 说明 |
|------|------|------|
| `1` | q-only 插值站起 | 关节空间 PD 插值至站立姿态 |
| `2` | MIT BALANCE_STAND | 四足站立平衡（QP-WBC） |
| `3` | MIT LOCOMOTION | 安全接管：站立→慢速踏步 |
| `x` | 回到 Q-STAND-HOLD | 返回关节保持模式 |
| `4` / `q` | 回趴 + disable + 退出 | 安全关机流程 |

---

## 7. 步态类型

项目支持丰富的步态模式，定义在 [Gait.cpp](user/MIT_Controller/Controllers/convexMPC/Gait.cpp)：

| 步态 | gaitNumber | 占空比 | 相位偏移 (FL/FR/HL/HR) | 描述 |
|------|------------|--------|------------------------|------|
| **Trot** | 0 | 0.5 | 0.0 / 0.5 / 0.5 / 0.0 | 对角小跑（默认 locomotion 步态） |
| **Standing** | 4 | 1.0 | 0.0 / 0.0 / 0.0 / 0.0 | 四足全站立 |
| **Bounding** | 1 | 0.5 | 0.0 / 0.0 / 0.5 / 0.5 | 前后腿结对跳跃 |
| **Pronking** | 2 | 0.4 | 0.0 / 0.0 / 0.0 / 0.0 | 四腿同时跳跃 |
| **Galloping** | 5 | 0.5 | 0.0 / 0.1 / 0.5 / 0.6 | 奔跑步态 |
| **Walking** | 6 | 0.75 | 0.0 / 0.5 / 0.25 / 0.75 | 静态行走 |
| **Pacing** | 7 | 0.5 | 0.0 / 0.5 / 0.0 / 0.5 | 同侧步态 |

---

## 8. 配置说明

### 8.1 机器人参数 (`config/legbot-defaults.yaml`)

```yaml
controller_dt: 0.001             # 控制周期 1kHz
cheater_mode: 1                  # 仿真中是否使用真实状态
stand_kp_cartesian: [80, 80, 80]  # 站立笛卡尔刚度
stand_kd_cartesian: [3.0, 3.0, 3.0]  # 站立笛卡尔阻尼
```

### 8.2 MPC 关键参数

定义在 `MIT_UserParameters.h` 和 `ConvexMPCLocomotion.cpp`：

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `horizonLength` | 10 | MPC 预测时域步数 |
| `iterationsBetweenMPC` | 30 | MPC 重新计算间隔（每 30 个控制周期 = 30ms） |
| `dtMPC` | ~0.03s | MPC 离散化时间步长（= 30 × controller_dt） |
| `_mu` | 0.6 | MPC 摩擦系数 |
| `_maxForce` | 250 N | MPC 单腿最大地面反力 |
| `use_wbc` | 1.0 | 是否启用 WBC（>0.9 启用） |

### 8.3 QP-WBC 关键参数

定义在 `LocomotionCtrl.cpp` 和 `QPWBC.cpp`：

| 参数 | 默认值 | LegBot 覆盖 | 含义 |
|------|--------|------------|------|
| `W_ori` | 300 | — | 身体姿态任务权重 |
| `W_pos` | 300 | Z轴: 10 | 身体位置任务权重 |
| `W_foot` | 100 | — | 摆动足跟踪任务权重 |
| `W_fr` | 1.0 | Z轴: 0.1 | 反力跟踪权重 |
| `W_qddot` | 0.001 | — | 广义加速度正则化权重 |
| `Kp_joint` / `Kd_joint` | 按关节配置 | — | 关节 PD 增益 |
| `maxFz` | 1500 N | — | WBC 单腿最大法向力 |
| `mu` | 0.4 | — | WBC 摩擦系数 |

> **LegBot Z 轴权重覆盖说明**：LegBot 实物在 Z 轴方向（高度控制）的响应特性与 MIT Cheetah 不同，将 `W_pos[2]` 降为 10（降低高度跟踪刚度）、`W_fr[2]` 降为 0.1（降低 Z 轴反力跟踪），避免实物高度振荡。

### 8.4 OSQP 求解器参数

| 参数 | 值 | 含义 |
|------|-----|------|
| `eps_abs` | 1e-3 | 绝对收敛容差 |
| `eps_rel` | 0 | 相对收敛容差（禁用） |
| `max_iter` | 1000 | 最大迭代次数 |
| `verbose` | false | 关闭日志输出 |
| `warm_start` | true | 启用 warm start |

---

## 9. 引用

本项目基于 [MIT Cheetah-Software](https://github.com/mit-biomimetics/Cheetah-Software) 进行深度二次开发，核心算法参考以下工作：

- **Convex MPC**: Di Carlo, J., Wensing, P. M., Katz, B., Bledt, G., & Kim, S. (2018). "Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control." *IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*.
- **WBIC**: Kim, D., Di Carlo, J., Katz, B., Bledt, G., & Kim, S. (2019). "Highly Dynamic Quadruped Locomotion via Whole-Body Impulse Control and Model Predictive Control." *arXiv preprint arXiv:1909.06586*.
- **Pinocchio**: Carpentier, J., et al. "The Pinocchio C++ library — A fast and flexible implementation of rigid body dynamics algorithms and their analytical derivatives." *SoftwareX*, 2019.
- **OSQP**: Stellato, B., Banjac, G., Goulart, P., Bemporad, A., & Boyd, S. (2020). "OSQP: An Operator Splitting Solver for Quadratic Programs." *Mathematical Programming Computation*, 12(4), 637-672.
- **Raibert Heuristic**: Raibert, M. H. (1986). "Legged Robots That Balance." *MIT Press*.

如果您使用本项目进行研究，请引用上述原始工作。

---

## 10. 致谢

- [MIT Biomimetics Robotics Lab](https://biomimetics.mit.edu/) — 原始 Cheetah-Software 框架
- [Robot-Nav](https://github.com/Robot-Nav) — LegBot 机器人的实机适配与算法扩展

---

<div align="center">
  <sub>Built with ❤ for quadruped robotics research</sub>
</div>

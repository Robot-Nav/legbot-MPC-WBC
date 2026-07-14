# LegBot 接入 Cheetah-Software 原算法栈方案

本文档描述 LegBot 使用 Cheetah-Software 原控制链、MuJoCo 只作为物理仿真后端的接入方案。`scripts/mujoco_legbot_walk.py` 仍然只作为 MuJoCo 模型和参数 baseline，不作为完整接入。

## 1. 当前状态

已存在的 LegBot 接入点：

- `common/include/cppTypes.h`：已有 `RobotType::LEGBOT`。
- `common/include/Dynamics/LegBot.h`：已有 `buildLegBot<T>()`，参数来自 `models/MJCF/legbot/legbot_mpc_scene.xml`。
- `config/legbot-defaults.yaml`：已有 LegBot robot control 参数，默认 `controller_dt: 0.001`。
- `config/legbot-mit-ctrl-user-parameters.yaml`：LegBot 专用 MIT_Controller user 参数，默认站立高度 `0.2775 m`，初始 gait 为 standing。
- `robot/src/main_helper.cpp`：已有命令行 robot-id `v`。
- `robot/src/RobotRunner.cpp`：已有 `buildLegBot<float>()`，LegBot 走 `SpiData/SpiCommand` 风格通路。
- `user/MIT_Controller/FSM_States/FSM_State_StandUp.cpp`：LegBot stand height 为 `0.2775 m`。
- `user/MIT_Controller/FSM_States/FSM_State_Locomotion.cpp`、`SafetyChecker.cpp`、`ConvexMPCLocomotion.cpp`：已有 LegBot 分支或 Mini/LegBot 共用分支。

尚未完成的关键点：

- 没有 `MuJoCo -> RobotRunner -> MuJoCo` 的 C++ bridge。
- 原 `robot/src/SimulationBridge.cpp` 仍是 shared-memory + Cheetah-Software 内置 simulator 通路。
- 原 `sim/src/Simulation.cpp` 即使支持 `RobotType::LEGBOT`，也使用 `DynamicsSimulator`，不加载 MJCF/STL。
- `scripts/mujoco_legbot_walk.py` 手写 Raibert/PD/MuJoCo Jacobian 控制器，绕过 `RobotRunner/FSM/ConvexMPC/WBC/LegController`。

## 2. 正确闭环

```text
models/MJCF/legbot/legbot_mpc_scene.xml
        |
        v
MuJoCo physics 1000 Hz: qpos/qvel/contact
        |
        v
MuJoCo-LegBot Bridge
  - freejoint base pose/velocity
  - joint q/dq
  - foot contact phase
  - IMU/cheater state
        |
        v
Cheetah-Software data
  SpiData + VectorNavData + CheaterState + GamepadCommand
        |
        v
RobotRunner
        |
        v
StateEstimator -> DesiredStateCommand -> ControlFSM
        |
        v
FSM_State_StandUp / FSM_State_Locomotion
        |
        v
ConvexMPCLocomotion -> WBC/LocomotionCtrl -> LegController
        |
        v
SpiCommand: tau_ff + joint PD gains/targets
        |
        v
MuJoCo data.ctrl
        |
        v
MuJoCo step
```

## 3. 推荐新增文件

最小 C++ bridge 推荐新增：

- `robot/include/MujocoLegBotBridge.h`
- `robot/src/MujocoLegBotBridge.cpp`
- `robot/src/mujoco_legbot_main.cpp`
- `robot/include/MujocoLegBotXml.h`
- `robot/src/MujocoLegBotXml.cpp`
- `config/legbot-mit-ctrl-user-parameters.yaml`

CMake 推荐做成可选目标：

```cmake
option(BUILD_MUJOCO_LEGBOT "Build LegBot MuJoCo backend" OFF)
if(BUILD_MUJOCO_LEGBOT)
  find_package(mujoco REQUIRED)
  add_executable(mujoco_legbot_bridge
    robot/src/mujoco_legbot_main.cpp
    robot/src/MujocoLegBotBridge.cpp
    robot/src/MujocoLegBotXml.cpp)
  target_link_libraries(mujoco_legbot_bridge robot biomimetics mujoco::mujoco)
endif()
```

这样缺 Qt5 或缺 MuJoCo C++ SDK 时，不影响 `mit_ctrl`、`robot`、`common` 目标。

## 4. MJCF 到 Quadruped 映射

源文件：`models/MJCF/legbot/legbot_mpc_scene.xml`。

| Quadruped 字段 | MJCF 来源 |
| --- | --- |
| `_bodyMass` | `body name="base"/inertial mass="9.016326"` |
| `_bodyInertia` | `base/inertial pos` 和 `fullinertia` |
| `_bodyLength/_bodyWidth/_bodyHeight` | base collision boxes 外包络，当前为 `0.50/0.205/0.138` |
| `_abadLocation` | `FR_hip pos="0.18453 -0.051 0"` 取绝对值后为 `(0.18453, 0.051, 0)` |
| `_abadLinkLength` | thigh body 相对 hip 的 y 偏移，`0.0975` |
| `_hipLinkLength` | calf body 相对 thigh 的 z 偏移，`0.1985` |
| `_kneeLinkLength` | foot body 相对 calf 的 z 偏移，`0.214` |
| `_abad/_hip/_kneeInertia` | 对应 hip/thigh/calf inertial mass、COM、diaginertia/quat 转换后的惯量 |
| `_motorTauMax` | hip/thigh `forcerange=-17 17`、calf `-34 34`；统一上限先取 `34`，bridge 写 ctrl 时按关节单独限幅 |
| `_jointDamping/_jointDryFriction` | MJCF default joint `damping="0.1"`、`frictionloss="0.2"` |

LegBot MJCF 腿名为 `FR/FL/RR/RL`。Cheetah-Software 代码注释常用 `FR/FL/HR/HL`，索引含义一致：

| Cheetah leg index | Cheetah 名 | LegBot MJCF 名 |
| --- | --- | --- |
| 0 | FR | FR |
| 1 | FL | FL |
| 2 | HR | RR |
| 3 | HL | RL |

关节映射：

| Cheetah joint | LegBot MJCF joint |
| --- | --- |
| abad | `${leg}_hip_joint`，axis x |
| hip | `${leg}_thigh_joint`，axis y |
| knee | `${leg}_calf_joint`，axis y |

## 5. MuJoCo 状态到 Cheetah 数据

bridge 每个 controller tick 填充：

```text
SpiData.q_abad[leg]  = data.qpos[jnt_qposadr(${leg}_hip_joint)]
SpiData.q_hip[leg]   = data.qpos[jnt_qposadr(${leg}_thigh_joint)]
SpiData.q_knee[leg]  = data.qpos[jnt_qposadr(${leg}_calf_joint)]
SpiData.qd_abad[leg] = data.qvel[jnt_dofadr(${leg}_hip_joint)]
SpiData.qd_hip[leg]  = data.qvel[jnt_dofadr(${leg}_thigh_joint)]
SpiData.qd_knee[leg] = data.qvel[jnt_dofadr(${leg}_calf_joint)]
```

base freejoint：

```text
qpos[base_qpos + 0:3] -> CheaterState.position
qpos[base_qpos + 3:7] -> CheaterState.orientation, MuJoCo wxyz 与 Cheetah quaternion wxyz 保持一致
qvel[base_qvel + 0:3] -> world linear velocity，需要转成 body velocity 后填 CheaterState.vBody
qvel[base_qvel + 3:6] -> world angular velocity，需要转成 body angular velocity 后填 CheaterState.omegaBody
```

IMU：

```text
VectorNavData.quat          = MuJoCo trunk_quat 转成 VectorNavOrientationEstimator 期望格式
VectorNavData.gyro          = body angular velocity
VectorNavData.accelerometer = body-frame acceleration, 不含或含重力要与 estimator 假设一致
```

最小闭环建议先设置 `cheater_mode: 1`，让 `RobotRunner::initializeStateEstimator(true)` 使用 `CheaterOrientationEstimator` 和 `CheaterPositionVelocityEstimator`，稳定站立后再切回 VectorNav + KF。

contact：

```text
for each mjContact:
  geom1/geom2 命中 foot geom name FR/FL/RR/RL 且另一侧为 ground
  contactPhase[leg] = 1.0
else
  contactPhase[leg] = 0.0
```

当前 `ContactEstimator` 会给默认估计，Locomotion 中 `ConvexMPCLocomotion` 也会按 gait 覆盖 contact phase。bridge 仍应保留真实 contact，后续可加一个 MuJoCo contact estimator 或让 `StateEstimatorContainer` 暴露外部 contact 注入接口。

## 6. Cheetah 命令到 MuJoCo ctrl

`RobotRunner::finalizeStep()` 后，LegBot 走 `LegController::updateCommand(SpiCommand*)`：

```text
tau = tau_ff
    + kp_joint * (q_des - q)
    + kd_joint * (qd_des - qd)
```

写 MuJoCo：

```text
data.ctrl[actuator(${leg}_hip_joint)]   = clamp(tau_abad,  -17, 17)
data.ctrl[actuator(${leg}_thigh_joint)] = clamp(tau_hip,   -17, 17)
data.ctrl[actuator(${leg}_calf_joint)]  = clamp(tau_knee,  -34, 34)
```

如果 Cheetah 输出中已有 Cartesian PD 转换出来的 `tau_ff`，bridge 不应再次做足端 Jacobian 控制；只做最终 joint-space PD 合成和限幅。

## 7. bridge 运行时序

建议参数：

- MuJoCo physics：`1000 Hz`，`timestep=0.001`。
- controller：先 `1000 Hz`，稳定后可试 `500 Hz`。
- viewer：`60 Hz`，即每 16 到 17 个 physics step sync 一次。

主循环：

```text
prepare_project_xml()
mj_loadXML()
RobotControlParameters.load(config/legbot-defaults.yaml)
MIT_UserParameters.load(config/legbot-mit-ctrl-user-parameters.yaml)
construct MIT_Controller + RobotRunner
bind pointers: SpiData/SpiCommand/VectorNavData/CheaterState/GamepadCommand/VisualizationData
RobotRunner.robotType = RobotType::LEGBOT
RobotRunner.init()

while running:
  read_mujoco_state_to_spi_and_estimator_inputs()
  if controller_tick:
    RobotRunner.run()
    write_spi_command_to_mujoco_ctrl()
  mj_step()
  if viewer_tick:
    viewer.sync()
```

## 8. StandUp 到 Locomotion 路线

阶段 1：StandUp

- `config/legbot-defaults.yaml`：`control_mode: K_STAND_UP` 对应数值需与 `ControlFSM` 常量一致。
- `cheater_mode: 1`。
- `use_rc: 0`，bridge 直接设置 `control_mode` 和期望速度，避免 SBUS 依赖。
- 验收：base height 收敛到 `0.2775 m` 附近，roll/pitch 小于 5 deg，12 个 actuator 无持续饱和。

阶段 2：Locomotion

- 切 `control_mode: K_LOCOMOTION`。
- 初始 `cmpc_gait: 4` standing，确认 WBC/MPC 输出稳定。
- 再切 trot，期望速度从 `0.3 m/s`、`0.5 m/s`、`1.0 m/s`、`1.5 m/s`、`2.0 m/s` 逐步扫描。

阶段 3：MPC/WBC 调参

- LegBot body height：`0.2775 m`。
- swing height：`0.05-0.07 m`。
- gait period：`0.50-0.60 s`，速度越高可降到 `0.42-0.48 s`。
- stance force limit：hip/thigh `17 Nm`、calf `34 Nm`，MPC 单腿垂向力先限制在 `120-180 N`。
- 前倾时：提高 pitch Q/Kp、减小前落脚 `pfx_rel`、增加 `cmpc_x_drag`，并检查 base COM x offset `0.026103 m` 是否在 MPC 惯量模型中考虑。
- 摔倒或腿抖：先关 sparse MPC，用 dense MPC；降低 swing Kp z；把 controller 从 500 Hz 临时切回 1000 Hz。

## 9. 推荐运行命令

当前可用 baseline：

```bash
conda run -n mujoco python scripts/mujoco_legbot_walk.py --prepare-xml
conda run -n mujoco python scripts/mujoco_legbot_walk.py --speed 1.0 --duration 8
```

原 Cheetah-Software 内置 sim 通路：

```bash
./build/user/MIT_Controller/mit_ctrl v s
```

真正 MuJoCo bridge 完成后推荐：

```bash
./build/robot/mujoco_legbot_bridge \
  --xml models/MJCF/legbot/legbot_mpc_scene.xml \
  --robot-params config/legbot-defaults.yaml \
  --user-params config/legbot-mit-ctrl-user-parameters.yaml \
  --mode standup \
  --physics-hz 1000 \
  --control-hz 1000 \
  --viewer-hz 60
```

## 10. 当前限制和后续优化

- 当前仓库还没有可执行的 C++ MuJoCo bridge；这是完整接入的主缺口。
- `RobotRunner::setupStep()` 当前为 private，单进程 bridge 最简单是直接调用 public `run()`，但初始化和参数加载需要 bridge 仿照 `SimulationBridge::runRobotControl()` 绑定数据指针。
- 若需要 bridge 在不走 LCM/shared-memory 的情况下强制切换 estimator，建议给 `RobotRunner` 增加一个明确的 simulation-input API，而不是让 bridge 访问内部私有成员。
- LegBot 的 sparse MPC 惯量和 max force 当前仍偏硬编码，后续应从 `buildLegBot()` 或 user params 注入。
- 内置 Qt sim 对 LegBot 使用 MiniCheetah 可视化 mesh，不能代表 MJCF 物理/视觉。
- Python baseline 已验证 MuJoCo 模型可跑，但不是 Cheetah-Software 算法验证。

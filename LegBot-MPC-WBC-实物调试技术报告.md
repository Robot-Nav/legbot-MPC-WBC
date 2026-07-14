# LegBot MPC+WBC 实物调试技术报告

> **项目目标**：在不修改 `ConvexMPCLocomotion / QP-WBC` 核心算法的前提下，将 Cheetah-Software 已经在仿真中验证过的 MIT_Controller 控制链路接入 LegBot 实物平台。当前阶段采用 DDS-only 架构：新 bridge 只订阅 `rt/lowstate`、必要时发布 `rt/lowcmd`，串口/CAN/IMU 仍由已经验证过的 `dds_to_serial_gateway` 独占管理。

---

## 目录

- [一、当前实物接入定位](#一当前实物接入定位)
- [二、系统进程与数据流](#二系统进程与数据流)
- [三、已实现功能](#三已实现功能)
- [四、关键文件与职责](#四关键文件与职责)
- [五、控制流程](#五控制流程)
- [六、实物调试命令](#六实物调试命令)
- [七、关键参数说明](#七关键参数说明)
- [八、当前测试结论](#八当前测试结论)
- [九、安全策略](#九安全策略)
- [十、下一步建议](#十下一步建议)
- [十一、当前进展快照](#十一当前进展快照)

---

## 一、当前实物接入定位

新实物 bridge 的核心原则是：

```text
ConvexMPC / WBC / LegController 正常计算
        ↓
SpiCommand 原始输出
        ↓
OutputSafetyGuard 对算法输出做保守 clip，或在显式 raw 模式下跳过命令侧 clip
        ↓
RuntimeSafetyGuard 检查真实反馈与 IMU
        ↓
检查通过：发布 LowCmd 到 rt/lowcmd
检查失败：停止算法输出 / disable / 正常返回
```

它不是一个新的 IK+PD 控制器，也不是 q-only stepping。key `2/3` 的目标是实测真实 MIT / MPC / WBC / LegController 输出链路。当前有两条实物输出路径：

```text
--real-output-guarded：FromSpiCommand() 后经过 OutputSafetyGuard，再 PublishLowCmd()
--real-output-raw：FromSpiCommand() 后直接 PublishLowCmd()，只保留反馈侧 RuntimeSafetyGuard
```

旧程序 `third-party/serial_dds_gateway/src/legbot_rt_gait_pd.cpp` 仍然作为 DDS 通信、安全策略和旧算法回归测试参考；新 bridge 的目标是验证 Cheetah-Software 中的 MIT standing / locomotion 算法在实物上的表现。

当前约束：

- 不直接访问 `/dev/myttyCAN0`、`/dev/myttyCAN1`、`/dev/myttyIMU`。
- 不修改 ConvexMPC / QP-WBC 的核心求解与控制律；key `3` 的实物接入保护放在 bridge 层完成，只读取现有调试快照和 LegController 输出。
- 不改变 MuJoCo 仿真路径。
- 实物输出只通过 `rt/lowcmd` 给 `dds_to_serial_gateway`。
- key `2` 不做 alpha blend；key `3` 只在从 gait=0 shadow 进入 direct 前做 bridge 层 q/qd/tau 接管 blend，不做 q-only stepping。
- key `2` 现在用于 `BALANCE_STAND` 站立接管；key `3` 才进入 `LOCOMOTION` 原地 slow-trot 接管流程。
- key `3` 当前采用固定安全接管流程：先让 Locomotion 在 gait=4 shadow 中后台 warmup，raw 通过检查后 gait=4 direct 真正接管站立，再 gait=0 shadow 验证 slow-trot raw，最后经 alpha blend 进入 gait=0 direct。

---

## 二、系统进程与数据流

### 2.1 实物通信拓扑

```text
电机 / CAN 串口 / IMU
        ↕
dds_to_serial_gateway
        ↕ DDS
rt/lowstate  /  rt/lowcmd
        ↕
legbot_real_bridge_dryrun
        ↕
LegBotDDSStateAdapter / LegBotDDSCommandAdapter
        ↕
SpiData / VectorNavData / CheaterState / SpiCommand
        ↕
RobotRunner
        ↕
MIT_Controller
        ↕
ControlFSM / ConvexMPC / WBC / LegController
```

### 2.2 与原 Cheetah-Software 的关系

新 bridge 复用了原框架中的主要链路：

| 原框架模块 | 当前用途 |
|---|---|
| `RobotRunner` | 周期调用 MIT_Controller，管理状态估计与控制器入口 |
| `MIT_Controller` | 使用原 ControlFSM、ConvexMPC、WBC、LegController |
| `SpiData` | 接收 LegBot DDS LowState 转换后的关节反馈 |
| `SpiCommand` | 承载 LegController 最终输出 |
| `VectorNavData` | 接收 DDS IMU 四元数和 gyro，并作为 `cheater_mode=0` 的真实估计器输入 |
| `CheaterState` | 仍填充以便调试，但在 `cheater_mode=0` / `initializeStateEstimator(false)` 下不参与主状态估计链路 |

---

## 三、已实现功能

### 3.1 LegBot 关节映射

文件：

```text
robot/include/LegBotDDS/LegBotJointMap.h
```

当前关节顺序：

```text
FR, FL, RR, RL
每条腿：hip, thigh, calf
```

已提供：

```text
ModelToDdsJoint(i)
DdsToModelJoint(i)
DdsQToModelQ()
ModelQToDdsQ()
DdsDqToModelDq()
ModelTauToDdsTau()
```

保留了 sign array，目前默认全 `+1`。如果后续发现实物方向与模型方向不一致，应优先在映射层修正，而不是修改 MPC/WBC 算法。

### 3.2 DDS 状态适配

文件：

```text
robot/include/LegBotDDS/LegBotDDSAdapters.h
```

`LegBotDDSStateAdapter` 已实现：

- 订阅 `rt/lowstate`。
- 读取 `motor_state[i].q / dq / lost`。
- 读取 IMU quaternion 和 gyroscope。
- DDS LowState IMU quaternion 记录为 `wxyz`。
- 填入 Cheetah `VectorNavData.quat` 时转换为 raw VectorNav 顺序 `xyzw`，因为 `VectorNavOrientationEstimator` 会再执行 `xyzw -> wxyz`。
- gyro 死区：
  - XY：`0.05 rad/s`
  - Z：`0.01 rad/s`
- 填充 Cheetah-Software 的 `SpiData` 和 `VectorNavData`。
- 保持 `cheater_mode=0` 和 `initializeStateEstimator(false)`，真实姿态估计走 `VectorNavOrientationEstimator + LinearKFPositionVelocityEstimator`。
- 提供 `LowState` timeout、motor lost 检查。
- dry-run 打印：
  - `dds_q`
  - `model_q`
  - `dds_dq`
  - `model_dq`
  - `imu_quat_wxyz`
  - `imu_rpy`
  - `gyro`
  - `lost`
  - `tick`

### 3.3 DDS 命令适配

文件：

```text
robot/include/LegBotDDS/LegBotDDSAdapters.h
```

`LegBotDDSCommandAdapter` 已实现：

```text
SpiCommand.q_des     -> LowCmd.motor_cmd[i].q
SpiCommand.qd_des    -> LowCmd.motor_cmd[i].dq
SpiCommand.kp        -> LowCmd.motor_cmd[i].kp
SpiCommand.kd        -> LowCmd.motor_cmd[i].kd
SpiCommand.tau_ff    -> LowCmd.motor_cmd[i].tau
```

支持：

- `dry-output`：只打印 `SpiCommand` 摘要，不发布 `LowCmd`。
- `real-output-guarded`：安全检查通过后发布 `rt/lowcmd`，带命令侧 OutputSafetyGuard clip。
- `real-output-raw`：发布算法 raw `rt/lowcmd`，不做命令侧 OutputSafetyGuard clip，但保留反馈侧 RuntimeSafetyGuard。
- 显式实验参数 `--tau-ff-scale`。
- disable burst。
- q-only stand-up / shutdown command。
- `OutputSafetyGuard`：
  - action/mode clip。
  - `q_des` 绝对范围 clip。
  - `q_des` 相邻周期 delta clip。
  - `qd_des` 相邻周期 delta clip。
  - `tau_ff` clip。
  - `Kp/Kd` max clip。
  - `Kp/Kd` 非有限值 fault。
  - `max_kp < 1` 或 `max_kd < 0.1` 时打印 `low_kp_kd`，首轮按 soft fault 停止发布算法输出。

### 3.4 安全监督器

文件：

```text
robot/include/LegBotDDS/LegBotSafetySupervisor.h
```

已实现模式：

```text
--dry-run
--dry-output
--real-output-guarded
--real-output-raw
--interactive-control
```

已实现运行时安全检查：

- `lowstate timeout`
- `lowstate tick stale`
- `motor_state[i].lost`
- `q / dq` 非有限值
- 真实反馈 q runtime limit
- 真实反馈 dq runtime limit
- 真实反馈 tau_est runtime limit
- 电机温度 warning / fault
- IMU quaternion 非有限值
- IMU quaternion norm 异常
- gyro 非有限值
- gyro 超限
- roll / pitch tilt 超限
- command `q_des / qd_des / tau_ff` 非有限值

当前策略是：

```text
guarded 模式命令侧 OutputSafetyGuard：clip 后继续发布，并打印 raw/pub 对比。
raw 模式命令侧不做 OutputSafetyGuard clip，只检查命令有限性。
反馈侧 RuntimeSafetyGuard：真实反馈越界后停止 MIT 输出、disable、正常 return。
```

命令侧 clip 不代表测试成功。若 clip 高频发生，说明算法输出与实物安全范围不匹配，需要回头检查腿序、零位、状态估计或 MPC/WBC 参数。

### 3.5 Bridge 可执行程序

文件：

```text
tools/legbot_real_bridge_dryrun.cpp
```

构建目标：

```text
build/user/MIT_Controller/legbot_real_bridge_dryrun
```

当前支持四类主流程：

| 模式 | 行为 |
|---|---|
| `--dry-run` | 只订阅 `rt/lowstate`，不创建 publisher，不发布 `LowCmd` |
| `--dry-output` | 跑 RobotRunner/MIT_Controller，只打印 `SpiCommand`，不发布 `LowCmd` |
| `--real-output-guarded` | 发布 guarded `LowCmd`，hard fault 时停止算法输出并 disable |
| `--real-output-raw` | 发布 raw `LowCmd`，不做命令侧 clip，反馈侧 hard fault 时停止输出并 disable |

附加交互模式：

```text
--interactive-control
```

键盘阶段：

```text
1  q-only 插值站起
2  MIT BALANCE_STAND direct
3  MIT LOCOMOTION shadow/warmup/blend takeover
x  回到 Q-STAND-HOLD
4  回趴 + disable + 退出
q  回趴 + disable + 退出
```

### 3.6 key 3 Locomotion shadow/warmup/blend 接入与 cmpc_debug

当前 key `3` 已改为 bridge 层六阶段接入，用于避免从 `BALANCE_STAND` 直接跳入 gait=0 trot 的对角摆腿相位。`RobotRunner / MIT_Controller / ControlFSM` 仍在程序启动时初始化；按 `3` 只是改变 bridge 的接管阶段，不延迟 new 控制器对象。

```text
1. MIT_FORWARD_ARMING
  control_mode = K_STAND_UP
  后台正常 runner->run()
  电机继续发布 key3_entry_q_hold / Q-STAND-HOLD
  时间型 warmup，默认约 0.5s
  不再等待 STAND_UP raw 的 kp/kd readiness
  超过 2s 会打印 warning 并强制进入 gait4 shadow

2. LOCOMOTION_GAIT4_SHADOW
  control_mode = K_LOCOMOTION
  cmpc_gait = 4
  vx/vy/yaw_rate = 0
  前 0.15s 做 Locomotion raw warmup，不累计 stable
  后台 runner->run()，记录 raw
  电机仍发布 key3_entry_q_hold，不发布 Locomotion raw
  连续 0.5s 通过 raw/mpc_table/swing/tilt/q jump 等检查后进入 direct
  standing gait 下 contactStates=0.5 只作为诊断，不作为 hard reject
  standing gait 下 foot_pdes jump 只作为诊断，不作为 hard reject

3. LOCOMOTION_GAIT4_DIRECT
  control_mode = K_LOCOMOTION
  cmpc_gait = 4
  vx/vy/yaw_rate = 0
  raw 先过 readiness，再过 OutputSafetyGuard
  电机发布 guarded Locomotion gait=4 输出
  当 --test-gait 4 时保持在 gait4 direct，不再自动进入 gait0
  当 --test-gait != 4 时，稳定约 3s 后进入 GAIT0_SHADOW

4. GAIT0_SHADOW
  control_mode = K_LOCOMOTION
  cmpc_gait = --test-gait
  vx/vy/yaw_rate = 0
  从 gait4 direct 切入前 reset ConvexMPC gait phase
  firstSwing 被置 true，swingTimeRemaining 清 0
  前 0.20s 做 gait0 raw warmup，不累计 stable
  后台 runner->run()，记录 gait=0 raw
  电机发布上一帧 gait4 direct 的安全命令，不发布 gait=0 raw
  保留 foot_target_jump 和 q_des_jump hard gate

5. GAIT0_BLEND
  raw 继续过 readiness 和 OutputSafetyGuard
  q / qd / tau 分别按 alpha_q / alpha_qd / alpha_tau 平滑接管

6. GAIT0_DIRECT
  control_mode = K_LOCOMOTION
  cmpc_gait = --test-gait
  vx/vy/yaw_rate = 0
  发布 guarded Locomotion gait=0 输出
```

新增 `cmpc_debug` 终端打印和 CSV 字段：

```text
mode
bridge_ready
bridge_reject_code
bridge_stage
alpha_q
alpha_qd
alpha_tau
requested_gait
cmpc_gait
control_mode
q_feedback_[0..11]
dq_feedback_[0..11]
q_des_raw_[0..11]
qd_des_raw_[0..11]
tau_raw_[0..11]
q_des_pub_[0..11]
qd_des_pub_[0..11]
tau_pub_[0..11]
foot_p_leg[0..3]_{x,y,z}
foot_pdes_leg[0..3]_{x,y,z}
forceff_leg[0..3]_{x,y,z}
cmpc_gait_number
cmpc_current_gait
cmpc_gait_phase
cmpc_iteration
cmpc_desired_contact_[0..3]
cmpc_swing_phase_[0..3]
cmpc_se_contact_phase_[0..3]
cmpc_swing_time_[0..3]
cmpc_swing_time_remaining_[0..3]
cmpc_mpc_table_now_[0..3]
cmpc_first_swing_[0..3]
```

验收目标：

```text
LOCOMOTION_GAIT4_SHADOW 阶段:
  mpc_table_now == [1,1,1,1]
  swing_phase ~= [0,0,0,0]
  swing_time ~= [0,0,0,0]
  desired_vx = 0
  q_des_raw 与 q_feedback 差值小
  RR/RL thigh q_des_raw >= 0.65
  contactStates 可为 [0.5,0.5,0.5,0.5]，仅打印诊断
  foot_pdes 与 foot_p 在 gait4 standing 下仅打印诊断，不作为 hard gate
  tilt < 0.10 rad

GAIT0_SHADOW 阶段:
  入口已 reset ConvexMPC gait phase
  前 0.20s gait0_raw_warmup 不累计 stable
  cmpc_swing_time 接近 0.26s
  第一帧 swing_phase 不直接接近 0.8
  foot_pdes 与 foot_p 轴向差值 < 0.08m
  RR/RL thigh q_des_raw >= 0.55
  max_abs_qd_des_raw <= 6.0 rad/s
  wbc_sum_Fr_z 不明显低于自重量级
```

---

## 四、关键文件与职责

| 文件 | 职责 |
|---|---|
| `tools/legbot_real_bridge_dryrun.cpp` | LegBot 实物 bridge 主程序，负责 DDS 初始化、RobotRunner 初始化、dry-run/dry-output/guarded/interactive 主循环 |
| `robot/include/LegBotDDS/LegBotJointMap.h` | model 与 DDS 关节顺序、符号、q/dq/tau 映射 |
| `robot/include/LegBotDDS/LegBotDDSAdapters.h` | LowState -> SpiData/IMU，SpiCommand -> LowCmd，stand/down 姿态，安全检查辅助 |
| `robot/include/LegBotDDS/LegBotSafetySupervisor.h` | 参数解析、运行模式、安全阈值、统计输出 |
| `user/MIT_Controller/MIT_Controller.hpp` | 暴露 ControlFSM / ConvexMPC 调试快照与 gait phase reset 接口 |
| `user/MIT_Controller/FSM_States/FSM_State_Locomotion.h` | 转发 ConvexMPC debug snapshot / resetGaitPhase |
| `user/MIT_Controller/Controllers/convexMPC/ConvexMPCLocomotion.h/.cpp` | 新增 `ConvexMPCDebugSnapshot` 与 gait phase reset；未改 MPC/WBC 核心算法 |
| `user/MIT_Controller/CMakeLists.txt` | 注册 `legbot_real_bridge_dryrun` 可执行目标 |
| `third-party/serial_dds_gateway/src/legbot_rt_gait_pd.cpp` | 旧 IK+PD gait 程序，作为 DDS 通信和安全参考 |
| `third-party/serial_dds_gateway` | 机械狗实物串口/CAN/IMU 到 DDS 的 gateway |

---

## 五、控制流程

### 5.1 dry-run

```text
启动 DDS
  ↓
订阅 rt/lowstate
  ↓
读取 LowState
  ↓
打印 q/dq/IMU/lost/tick
  ↓
不创建 LowCmd publisher
```

用途：

- 验证 gateway 是否正常发布 `rt/lowstate`。
- 验证 motor lost 是否消失。
- 验证关节映射后的 `model_q` 是否合理。
- 验证 IMU roll/pitch/gyro 是否稳定。

### 5.2 dry-output

```text
LowState DDS
  ↓
LegBotDDSStateAdapter
  ↓
SpiData / VectorNavData / CheaterState
  ↓
RobotRunner
  ↓
MIT_Controller / ControlFSM / ConvexMPC / WBC
  ↓
SpiCommand
  ↓
打印命令摘要
  ↓
不发布 LowCmd
```

用途：

- 验证 Cheetah 控制链能否跑通。
- 检查 `SpiCommand` 是否出现 NaN、全 0、异常大输出。
- 不驱动电机。

### 5.3 real-output-guarded 自动流程

```text
等待 rt/lowstate
  ↓
bootstrap motor feedback
  ↓
q-only stand-up: 当前 q -> stand q
  ↓
FSM warmup: 请求 K_STAND_UP
  ↓
controller output: 请求 K_BALANCE_STAND 或 K_LOCOMOTION，取决于测试入口
  ↓
test locomotion: 请求 K_LOCOMOTION + test gait/velocity
  ↓
正常完成：shutdown ramp 当前 q -> down q
  ↓
disable burst
```

注意：之前实测出现过：

```text
CONTROL FSM Bad Request: Cannot transition from 0 to 4
```

原因是 FSM 仍在 `PASSIVE=0`，bridge 直接请求 `LOCOMOTION=4`。当前已经加入：

```text
--fsm-warmup-seconds
```

先请求 `K_STAND_UP=1`，再按测试入口切入 `K_BALANCE_STAND=3` 或 `K_LOCOMOTION=4`。

### 5.4 real-output-guarded 交互流程

交互模式适合实物调试：

```text
启动 bridge
  ↓
bootstrap motor feedback
  ↓
等待键盘
  ↓
按 1：q-only 插值站起
  ↓
按 2：MIT BALANCE_STAND direct
  ↓
按 3：MIT LOCOMOTION shadow/warmup/blend takeover
  ↓
按 4/q：回趴并 disable
```

这个流程比固定时间自动流程更适合调试，因为每一步都可以人工观察后再继续。

key `2/3` 的真实发布链路是：

```text
DDS LowState
  -> SpiData / VectorNavData
  -> RobotRunner::run()
  -> MIT_Controller::runController()
  -> ControlFSM::runFSM()
  -> key 2: FSM_State_BalanceStand / WBC / LegController
  -> key 3: FSM_State_Locomotion / ConvexMPCLocomotion / WBC / LegController
  -> SpiCommand
  -> FromSpiCommand()
  -> OutputSafetyGuard
  -> LowCmd
  -> PublishLowCmd(rt/lowcmd)
```

key `2/3` 都带有 MIT 输出 warmup / valid-output 等待阶段：

```text
先请求目标 FSM
继续发布 Q-STAND-HOLD
后台连续跑 RobotRunner / MIT_Controller
检查 SpiCommand 是否 finite 且不再是无效全 0 输出
有效后才进入 direct 发布
超时则 soft fault 回 Q-STAND-HOLD
```

key `3` 当前不是直接 `LOCOMOTION + gait=0`，而是固定走 Locomotion shadow / direct / blend 安全接管：

```text
按 3
  ↓
若当前已在 BALANCE_STAND：
  control_mode = K_LOCOMOTION
  进入 LOCOMOTION_GAIT4_SHADOW
否则：
  先 K_STAND_UP warmup
  再 control_mode = K_LOCOMOTION
  进入 LOCOMOTION_GAIT4_SHADOW
  ↓
LOCOMOTION_GAIT4_SHADOW:
  cmpc_gait = 4
  vx/vy/yaw_rate = 0
  后台运行 Locomotion，记录 raw
  实际发布 key3_entry_q_hold
  等待 raw/contact/swing/foot/tilt 连续稳定
  ↓
LOCOMOTION_GAIT4_DIRECT:
  cmpc_gait = 4
  vx/vy/yaw_rate = 0
  raw 先过 readiness，再过 OutputSafetyGuard
  发布 guarded gait=4 输出，约 3s
  ↓
GAIT0_SHADOW:
  cmpc_gait = --test-gait
  vx/vy/yaw_rate = 0
  后台运行 gait=0，记录 raw
  实际发布上一帧 gait4 direct 安全命令
  ↓
GAIT0_BLEND:
  q / qd / tau 使用 smoothstep alpha 分别在 2.0 / 2.5 / 3.0s 内接管
  ↓
GAIT0_DIRECT:
  发布 guarded gait=0 Locomotion 输出
```

这个改动的目的不是改变 ConvexMPC trot 算法，而是在 bridge 层确保 Locomotion 先算出合理的 gait=4 standing raw，再让 gait=4 真正闭环站稳，然后验证 gait=0 raw，最后逐步把 gait=0 输出交给电机。

这里仍然没有：

```text
q-only stepping
修改 ConvexMPC / QP-WBC 核心求解器
修改腿序 / IMU / DDS gateway
用另一个控制器替代 MIT/MPC/WBC
```

这里新增了：

```text
shadow 阶段：后台运行算法并记录 raw，但电机发布安全 hold / last safe command
blend 阶段：只对已经通过 readiness + OutputSafetyGuard 的 gait=0 raw 做 q/qd/tau 接管比例
```

---

## 六、实物调试命令

### 6.1 启动 DDS 串口 gateway

在 OrangePi 终端 1：

```bash
cd ~/fatuDog/serial_dds_gateway

./build/dds_to_serial_gateway \
  --serial-port-a /dev/myttyCAN0 \
  --serial-port-b /dev/myttyCAN1 \
  --baudrate 2000000 \
  --network lo \
  --tick-hz 100 \
  --imu-port /dev/myttyIMU \
  --imu-baudrate 921600 \
  --imu-gyro-deadzone 0.005
```

需要观察：

```text
rx_frames
type2_frames
tx_type1
tx_enable
decode_errors
tx_write_errors
imu_frames
```

正常现象：

- 电机未使能或未通电时，`rx_frames/type2_frames` 可能为 0，bridge 看到 `lost=1`。
- 电机通电并收到有效命令后，`rx_frames/type2_frames` 增长，`lost=none`。
- `decode_errors` 应尽量保持 0 或很少。
- `tx_write_errors` 应为 0。

### 6.2 编译 bridge

在 OrangePi：

```bash
cd ~/Cheetah-Software-master
cmake --build build --target legbot_real_bridge_dryrun -j4
```

如果本机修改后需要同步：

```bash
cd /home/lushilin/Cheetah-Software-master

rsync -avR \
  ./robot/include/LegBotDDS/LegBotSafetySupervisor.h \
  ./tools/legbot_real_bridge_dryrun.cpp \
  ./user/MIT_Controller/Controllers/convexMPC/ConvexMPCLocomotion.h \
  ./user/MIT_Controller/Controllers/convexMPC/ConvexMPCLocomotion.cpp \
  ./user/MIT_Controller/FSM_States/FSM_State_Locomotion.h \
  ./user/MIT_Controller/MIT_Controller.hpp \
  orangepi@192.168.3.93:~/Cheetah-Software-master/
```

### 6.3 dry-run 测试

```bash
cd ~/Cheetah-Software-master

./build/user/MIT_Controller/legbot_real_bridge_dryrun \
  --dry-run \
  --network lo \
  --duration 10 \
  --cmd-hz 100
```

验收点：

```text
lost=none
tick 持续增长
dds_q/model_q 合理
imu_rpy roll/pitch 合理
gyro 静止时接近 0
```

### 6.4 dry-output 测试

```bash
./build/user/MIT_Controller/legbot_real_bridge_dryrun \
  --dry-output \
  --network lo \
  --duration 5 \
  --cmd-hz 100 \
  --robot-standing-supported \
  --i-accept-risk
```

验收点：

- RobotRunner 初始化成功。
- FSM 不应一直报非法切换。
- `SpiCommand` 摘要不应长期全 0。
- 不会发布 `rt/lowcmd`。

### 6.5 自动 real-output-guarded 测试

```bash
./build/user/MIT_Controller/legbot_real_bridge_dryrun \
  --real-output-guarded \
  --test-locomotion \
  --network lo \
  --duration 4 \
  --cmd-hz 100 \
  --standup-ramp-seconds 6 \
  --standup-prehold-seconds 2 \
  --shutdown-ramp-seconds 6 \
  --shutdown-prehold-seconds 1 \
  --standup-kp 50 \
  --standup-kd 3 \
  --fsm-warmup-seconds 0.5 \
  --test-gait 0 \
  --test-vx 0.0 \
  --test-vy 0.0 \
  --test-yaw-rate 0.0 \
  --tau-ff-scale 1.0 \
  --cmd-action-limit 100 \
  --cmd-hip-abs-limit 0.55 \
  --cmd-thigh-min 0.45 \
  --cmd-thigh-max 1.35 \
  --cmd-calf-min -2.75 \
  --cmd-calf-max -1.20 \
  --cmd-qdes-delta-limit 0.03 \
  --cmd-qd-delta-limit 0.5 \
  --cmd-tau-limit 12.0 \
  --cmd-kp-max 80 \
  --cmd-kd-max 6 \
  --fb-hip-abs-limit 0.70 \
  --fb-thigh-min 0.30 \
  --fb-thigh-max 1.50 \
  --fb-calf-min -2.90 \
  --fb-calf-max -1.00 \
  --fb-tau-limit 30.0 \
  --fb-qd-limit 20.0 \
  --fb-temp-warning 70.0 \
  --fb-temp-limit 80.0 \
  --max-gyro-rad-s 5.0 \
  --max-tilt-rad 0.35 \
  --lowstate-timeout-s 0.25 \
  --disable-on-exit \
  --robot-standing-supported \
  --i-accept-risk
```

总时长约：

```text
bootstrap 若干秒
+ standup_ramp 6s
+ fsm_warmup 0.5s
+ controller stand 2s
+ locomotion duration 4s
+ shutdown ramp 6s
+ shutdown prehold 1s
```

### 6.6 推荐交互测试

```bash
./build/user/MIT_Controller/legbot_real_bridge_dryrun \
  --real-output-guarded \
  --interactive-control \
  --network lo \
  --duration 0 \
  --cmd-hz 100 \
  --standup-ramp-seconds 6 \
  --shutdown-ramp-seconds 6 \
  --shutdown-prehold-seconds 1 \
  --standup-kp 50 \
  --standup-kd 3 \
  --fsm-warmup-seconds 0.5 \
  --test-gait 4 \
  --algo-forward-vx 0.0 \
  --tau-ff-scale 1.0 \
  --cmd-action-limit 100 \
  --cmd-hip-abs-limit 0.55 \
  --cmd-thigh-min 0.45 \
  --cmd-thigh-max 1.35 \
  --cmd-calf-min -2.75 \
  --cmd-calf-max -1.20 \
  --cmd-qdes-delta-limit 0.03 \
  --cmd-qd-delta-limit 0.5 \
  --cmd-tau-limit 12.0 \
  --cmd-kp-max 80 \
  --cmd-kd-max 6 \
  --fb-hip-abs-limit 0.70 \
  --fb-thigh-min 0.30 \
  --fb-thigh-max 1.50 \
  --fb-calf-min -2.90 \
  --fb-calf-max -1.00 \
  --fb-tau-limit 30.0 \
  --fb-qd-limit 20.0 \
  --fb-temp-warning 70.0 \
  --fb-temp-limit 80.0 \
  --max-gyro-rad-s 5.0 \
  --max-tilt-rad 0.35 \
  --lowstate-timeout-s 0.25 \
  --disable-on-exit \
  --robot-standing-supported \
  --i-accept-risk
```

键盘：

```text
1  q-only 插值站起
2  MIT BALANCE_STAND direct + OutputSafetyGuard
3  MIT LOCOMOTION gait4 shadow/direct -> gait0 shadow/blend/direct + OutputSafetyGuard
x  回到 Q-STAND-HOLD
4  回趴 + disable + 退出
q  回趴 + disable + 退出
```

key `3` 首轮建议先保守：

```bash
--test-gait 4 \
--algo-forward-vx 0.0
```

先单独确认 `LOCOMOTION_GAIT4_SHADOW -> LOCOMOTION_GAIT4_DIRECT`，并让 gait4 direct 稳定 5~10s。通过后再单独用：

```bash
--test-gait 0 \
--algo-forward-vx 0.0
```

进入 `GAIT0_SHADOW`，确认 raw q/qd、foot_pdes、wbc_sum_Fr_z 合理后，再考虑 `GAIT0_BLEND` 和速度命令。目前 bridge 的 key `3` 实现对 gait=4/gait=0 都强制 `vx/vy/yaw_rate=0`，`--algo-forward-vx` 暂不参与 key `3` 发布路径。

```bash
--algo-forward-vx 0.03
```

### 6.7 raw stepping 首轮观察

当目标是观察算法原始 stepping 输出，而不是验证 OutputSafetyGuard clip 能否挡住异常输出时，使用显式 raw 模式：

```bash
./build/user/MIT_Controller/legbot_real_bridge_dryrun \
  --real-output-raw \
  --interactive-control \
  --network lo \
  --duration 0 \
  --cmd-hz 100 \
  --standup-ramp-seconds 8 \
  --shutdown-ramp-seconds 8 \
  --shutdown-prehold-seconds 1 \
  --standup-kp 50 \
  --standup-kd 3 \
  --fsm-warmup-seconds 0.5 \
  --mit-output-wait-seconds 2.0 \
  --test-gait 0 \
  --algo-forward-vx 0.0 \
  --tau-ff-scale 0.0 \
  --fb-hip-abs-limit 0.80 \
  --fb-thigh-min 0.20 \
  --fb-thigh-max 1.60 \
  --fb-calf-min -3.00 \
  --fb-calf-max -0.90 \
  --fb-tau-limit 35.0 \
  --fb-qd-limit 25.0 \
  --fb-temp-warning 70.0 \
  --fb-temp-limit 80.0 \
  --max-gyro-rad-s 8.0 \
  --max-tilt-rad 0.35 \
  --lowstate-timeout-s 0.25 \
  --disable-on-exit \
  --robot-standing-supported \
  --i-accept-risk
```

说明：

```text
--real-output-raw 明确创建 LowCmd publisher 并发布算法 raw 输出。
不要用“省略 --real-output-guarded”表达裸发；省略后会落到 dry-run / dry-output 语义，不发布 LowCmd。
raw 模式没有 cmd-qdes-delta-limit、cmd q_des 范围、cmd-tau-limit、cmd-kp/kd clip。
raw 模式仍保留反馈侧 q/dq/tau/temp/IMU/timeout hard fault。
```

首轮按键建议：

```text
1  q-only stand-up，等稳定
2  BALANCE_STAND raw，确认站稳 2~3 秒
3  LOCOMOTION shadow/blend/raw，观察阶段日志和 raw/pub 差异；必要时 0.5~1 秒内按 x
x  立刻回 Q-STAND-HOLD
4  down + disable
```

raw 模式风险明显高于 guarded。key `3` 只用于短时间观察腿是否按预期关系动作、是否某条腿大幅抽动、身体是否瞬间侧压。

---

## 七、关键参数说明

### 7.1 模式参数

| 参数 | 含义 |
|---|---|
| `--dry-run` | 只读 `rt/lowstate`，不创建 `LowCmdPublisher` |
| `--dry-output` | 跑 RobotRunner/MIT_Controller，只打印 `SpiCommand` |
| `--real-output-guarded` | 安全检查通过后发布 `rt/lowcmd` |
| `--real-output-raw` | 发布 raw 算法 `rt/lowcmd`，不做命令侧 clip，只保留反馈侧安全 |
| `--interactive-control` | 启用键盘分阶段控制 |

### 7.2 站起与回趴

| 参数 | 当前建议 | 含义 |
|---|---:|---|
| `--standup-ramp-seconds` | `6 ~ 10` | 当前 q 插值到 stand q |
| `--standup-kp` | `50` | q-only 站起 PD kp，当前实物已验证能站住 |
| `--standup-kd` | `3` | q-only 站起 PD kd |
| `--shutdown-ramp-seconds` | `6 ~ 8` | 当前 q 插值到 down q |
| `--shutdown-prehold-seconds` | `1 ~ 2` | down 姿态保持后 disable |

当前写入的模型角度：

```text
stand:
  hip   0.0
  thigh 0.9
  calf -1.8

down:
  FR [-0.02, 1.08, -2.64]
  FL [ 0.03, 1.08, -2.64]
  RR [-0.05, 1.08, -2.64]
  RL [ 0.06, 1.08, -2.64]
```

### 7.3 MIT FSM 与步态

| 参数 | 含义 |
|---|---|
| `--fsm-warmup-seconds` | 在切入 `BALANCE_STAND` 或 `LOCOMOTION` 前先请求 `K_STAND_UP=1` 的时间 |
| `--mit-output-wait-seconds` | 切入目标 FSM 后后台跑 MIT 输出、但继续发布 Q-STAND-HOLD，等待 `SpiCommand` 有效的超时时间 |
| `--test-gait` | 写入 `MIT_UserParameters::cmpc_gait` |
| `--locomotion-allstance-seconds` | 历史 all-stance direct 参数；当前 key `3` 已改为固定 gait4 shadow/direct，交互 key `3` 不再依赖该参数决定流程 |
| `--locomotion-ramp-seconds` | 历史 direct 输出层 ramp 参数；当前 key `3` 使用固定 alpha_q/alpha_qd/alpha_tau bridge blend |
| `--test-command-ramp-seconds` | 历史自动测试参数；当前 key `3` 交互接管 blend 使用代码内固定 2.0/2.5/3.0s |

已知 FSM 约束：

```text
PASSIVE=0 不能直接跳 LOCOMOTION=4
PASSIVE=0 可以跳 STAND_UP=1
STAND_UP=1 可以跳 BALANCE_STAND=3
STAND_UP=1 可以跳 LOCOMOTION=4
```

### 7.4 key 3 slow-trot 接管参数

| 参数 | 当前建议 | 含义 |
|---|---:|---|
| `--algo-forward-vx` | `0.0` | 当前 key `3` bridge 强制 gait4/gait0 的 vx/vy/yaw_rate 为 0；该参数保留但暂不参与 key `3` 发布路径 |
| `--test-gait` | `4 -> 0` | 第一轮用 4 只验证 gait4 standing direct；通过后第二轮改为 0，进入 GAIT0_SHADOW / BLEND / DIRECT |

当前不再使用 `w/s/a/d` 做交互速度调节。key `3` 的 `LOCOMOTION_GAIT4_SHADOW / DIRECT` 和 `GAIT0_SHADOW / BLEND / DIRECT` 都强制 `vx=0, vy=0, yaw_rate=0`。当前顺序是先验证 gait4 standing direct，再验证原地 slow-trot raw 是否连续。

首轮建议：

```text
--test-gait 4
--algo-forward-vx 0.0
```

gait4 direct 稳定 5~10s 后，再开第二轮：

```text
--test-gait 0
--algo-forward-vx 0.0
```

需要重点看：

```text
cmpc_desired_contact_[0..3]
cmpc_swing_phase_[0..3]
cmpc_mpc_table_now_[0..3]
cmpc_first_swing_[0..3]
state_vworld_x
num_qdes_delta_clamped
num_qd_delta_clamped
```

### 7.5 OutputSafetyGuard 命令侧 clip

这些限制只作用于 key `2/3` 的算法输出保护，不影响 key `4/q` 的正常 down ramp。

| 参数 | 首轮默认 | 含义 |
|---|---:|---|
| `--cmd-action-limit` | `100` | action/mode 上限保护；当前 LowCmd 主要使用 mode 0/1 |
| `--cmd-hip-abs-limit` | `0.55` | hip `q_des` 绝对值上限 |
| `--cmd-thigh-min` | `0.45` | thigh `q_des` 下限 |
| `--cmd-thigh-max` | `1.35` | thigh `q_des` 上限 |
| `--cmd-calf-min` | `-2.75` | calf `q_des` 下限，允许覆盖 down/prone 附近的 `-2.64` |
| `--cmd-calf-max` | `-1.20` | calf `q_des` 上限 |
| `--cmd-qdes-delta-limit` | `0.03` | `q_des` 相邻周期跳变限制，100Hz 下约 3 rad/s |
| `--cmd-qd-delta-limit` | `0.5` | `qd_des` 相邻周期跳变限制 |
| `--cmd-tau-limit` | `12.0` | `tau_ff` 首轮默认限幅 |
| `--cmd-kp-min` | `0` | 不强行抬高 Kp 下限，避免掩盖算法问题 |
| `--cmd-kp-max` | `80` | Kp 上限 |
| `--cmd-kd-min` | `0` | 不强行抬高 Kd 下限 |
| `--cmd-kd-max` | `6` | Kd 上限 |

说明：

```text
stand q 约为 [0, 0.9, -1.8]
down/prone q 的 calf 约为 -2.64
```

因此命令侧 calf 下限设置为 `-2.75`，不会挡住正常下趴附近姿态。`tau_ff` 首轮默认 `12.0 Nm`，不使用 `40 Nm` 作为首轮默认值；`40 Nm` 只适合作为后续稳定后的放宽测试值。

如果 MIT 输出出现：

```text
max_kp < 1
max_kd < 0.1
```

bridge 会打印：

```text
[LEGBOT-DDS][OUTPUT-GUARD][WARN] low_kp_kd
```

首轮按 soft fault 处理，不继续发布算法输出。

### 7.6 RuntimeSafetyGuard 反馈侧 fault

反馈侧使用真实 LowState，不做 clip，只作为最后安全底线。

| 参数 | 首轮默认 | 含义 |
|---|---:|---|
| `--lowstate-timeout-s` | `0.25` | DDS LowState 超时 |
| `--max-tilt-rad` | `0.35` | roll/pitch 最大倾角 |
| `--max-gyro-rad-s` | `5.0` | gyro 瞬时最大值 |
| `--fb-hip-abs-limit` | `0.70` | 真实反馈 hip 绝对值上限 |
| `--fb-thigh-min` | `0.30` | 真实反馈 thigh 下限 |
| `--fb-thigh-max` | `1.50` | 真实反馈 thigh 上限 |
| `--fb-calf-min` | `-2.90` | 真实反馈 calf 下限 |
| `--fb-calf-max` | `-1.00` | 真实反馈 calf 上限 |
| `--fb-tau-limit` | `30.0` | 真实反馈 `tau_est` 上限 |
| `--fb-qd-limit` | `20.0` | 真实反馈 `dq` 上限 |
| `--fb-temp-warning` | `70.0` | 电机温度 warning |
| `--fb-temp-limit` | `80.0` | 电机温度 fault |

反馈侧 q limit 比命令侧 q_des limit 稍宽。命令侧先 clip，反馈侧是最后安全底线。

反馈侧 fault 后：

```text
停止发布 MIT 算法输出
发送 disable / Passive
exit_reason=runtime_safety_failed:<reason>
正常 return
不执行 prone ramp
不 throw
不 Aborted
```

### 7.7 tau_ff_scale

```text
--tau-ff-scale
```

这是显式实验参数，不是安全层默认削弱算法。

当前首轮 direct 测试已经由 `--cmd-tau-limit 12.0` 负责保守力矩限幅。`--tau-ff-scale` 默认可以保持 `1.0`，除非需要单独排查 tau_ff 对动作的影响。

前提：

- 关节映射正确。
- sign 正确。
- stand/down 角度与实物一致。
- `SpiCommand` 没有 NaN/Inf。
- BALANCE_STAND 能稳定保持。

---

## 八、当前测试结论

### 8.1 编译与架构

当前目标已经能在 OrangePi 编译：

```text
[100%] Built target legbot_real_bridge_dryrun
```

此前遇到的架构不一致问题，主要来自把 x86 build 产物带到 ARM64。正确做法是在 OrangePi 上删除旧 build 后重新 CMake/build。

### 8.2 DDS 与电机反馈

已验证：

- `twelve_motor_serial` 在电机通电后可以收到 12 个电机反馈。
- `rx_frames/type2_frames` 会增长。
- 电机没通电或没 enable 时，`motor_state[i].lost()` 会全 1。
- gateway 本身不等 `rt/lowcmd` 也会发布 `rt/lowstate`，但电机反馈有效依赖电机侧状态和命令交互。

关键结论：

```text
motor_state[i].lost=1 不一定是 DDS 订阅失败；
它更可能表示 gateway 没有收到电机 type2 feedback。
```

### 8.3 IMU

已实现：

- DDS LowState quaternion 在日志中按 `wxyz` 打印。
- 填入 Cheetah `VectorNavData.quat` 时使用 `xyzw`，匹配 `VectorNavOrientationEstimator` 的真实 VectorNav 输入假设。
- 保持 `robot_params.cheater_mode = 0` 和 `runner.initializeStateEstimator(false)`，实机部署路径使用真实 IMU / 关节反馈状态估计。
- `FillCheaterFromLowState()` 仍保留用于调试，但当前主链路不依赖 CheaterState。
- gyro 加死区：
  - XY `0.05`
  - Z `0.01`

已定位过的关键问题：

```text
DDS 外层 imu_rpy roll/pitch 正常
Cheetah 内部 Orientation safety 出现异常大 roll
```

该现象由四元数顺序不匹配解释：DDS 是 `wxyz`，Cheetah `VectorNavOrientationEstimator` 期望 raw VectorNav `xyzw`，内部再转为 `wxyz`。当前 `FillCheetahData()` 已修正为：

```text
DDS wxyz -> VectorNavData xyzw
```

如果后续仍出现异常 `vBody/vWorld`，下一步再查 LinearKF 的真实加速度、足端运动学、contactEstimate 和 base height 输入。

### 8.4 站起/回趴

已验证 q-only stand-up 能让机器从当前姿态插值到 stand，并在实物上站住。

当前实测通过配置：

```text
standup_kp = 50
standup_kd = 3
stand_q_per_leg = [0, 0.9, -1.8]
```

此前出现过站立后直接 disable 的危险行为，因此已经加入：

```text
当前姿态 -> down q
down prehold
disable burst
```

用户主动退出时执行：

```text
当前 q -> down/prone q
down prehold
disable burst
```

但 runtime hard fault 时不再执行 prone ramp，避免在已经倾倒、gyro 超限或反馈异常时继续尝试姿态插值。hard fault 路径改为：

```text
停止 MIT 算法输出
current-q damping hold 0.2~0.5s
disable burst
正常 return
```

### 8.5 FSM 问题

实测日志出现：

```text
CONTROL FSM Bad Request: Cannot transition from 0 to 4
```

原因：

```text
ControlFSM 处于 PASSIVE=0
bridge 直接请求 LOCOMOTION=4
该跳转不被 FSM_State_Passive 允许
```

已修正：

```text
先请求 K_STAND_UP=1
等待 --fsm-warmup-seconds
再按按键入口请求 K_BALANCE_STAND=3 或 K_LOCOMOTION=4
```

### 8.6 key 2 旧 MIT LOCOMOTION standing 入口不适合首轮站立接管

旧版本 key `2` 直接切 MIT LOCOMOTION standing gait，实物测试失败：

```text
按 2 后机器人直接倒
日志触发 max_gyro_rad_s
随后 throw std::runtime_error("max_gyro_rad_s")
进程 Aborted
```

这说明：

```text
MIT/MPC/WBC 输出链路必须真实测试；
但 key 2 不应该继续用 LOCOMOTION + cmpc_gait=4 承担站立接管。
```

当前修正方向：

```text
key 2 = MIT BALANCE_STAND direct
key 3 = MIT LOCOMOTION shadow/warmup/blend takeover
```

原因：

```text
LOCOMOTION 即使 vx=0 / cmpc_gait=4，内部仍会经过 gait / swing / foot trajectory / WBC locomotion 逻辑。
实测曾出现 raw q_des 周期性跳到 thigh=4.7、calf=-8.4 一类异常站立目标。
OutputGuard 能挡住这类命令，但这不代表 standing 控制入口正确。
BALANCE_STAND 才是 all-stance standing 入口：四腿 contact=true，Fr_des.z=body_weight/4，通过 WBC/LegController 做站立控制。
```

`real-output-guarded` 模式下 OutputSafetyGuard 对算法输出做安全 clip。key `3` 的 `GAIT0_BLEND` 在 OutputSafetyGuard 之后逐步放开 q/qd/tau，不替换成 q-only，不做 q-only stepping。`real-output-raw` 模式下跳过命令侧 clip，但 key `3` 的 shadow / blend 阶段仍由 bridge 控制实际发布内容，用于短时间观察 raw stepping。

### 8.7 当前模型角与关节顺序结论

当前 DDS 输入和 LowCmd 输出都使用模型角，bridge 不再额外做腿序或符号转换。

当前顺序：

```text
leg 0 = FR
leg 1 = FL
leg 2 = RR
leg 3 = RL

每条腿:
0 = hip / abad
1 = thigh / hip
2 = calf / knee
```

12 维顺序：

```text
FR_hip, FR_thigh, FR_calf,
FL_hip, FL_thigh, FL_calf,
RR_hip, RR_thigh, RR_calf,
RL_hip, RL_thigh, RL_calf
```

`LegBotJointMap.h` 当前是 identity map：

```text
ModelToDdsMap = {0,1,2,3,4,5,6,7,8,9,10,11}
JointSigns = {+1 ... +1}
```

需要注意：MIT/MPC/WBC 运行时不直接读取 `/models` 目录里的 URDF/MJCF，而是 `RobotRunner::init()` 根据 `RobotType::LEGBOT` 调用 `buildLegBot<float>()`，使用 `common/include/Dynamics/LegBot.h` / `common/src/Dynamics/LegBot.cpp` 生成的 `Quadruped`。

当前 stand q 与 DDS / model 顺序一致：

```text
FR [-0.0, 0.9, -1.8]
FL [ 0.0, 0.9, -1.8]
RR [-0.0, 0.9, -1.8]
RL [ 0.0, 0.9, -1.8]
```

### 8.8 tau_ff 与 guarded/raw 测试结论

实测观察：

```text
tau_ff_scale=0 时，q_des + kp/kd 路径可以维持站立，tilt 约 0.01 rad 量级。
tau_raw/tau_pub 为 0 时仍能站住，说明 DDS 模型角输入输出链路基本正确。
此前侧倒与 WBC/MPC tau_ff 或 locomotion 入口输出强相关。
```

结论：

```text
继续收紧 OutputGuard 不是根治；
key 2 应先用 BALANCE_STAND 验证 all-stance standing；
key 3 再进入 LOCOMOTION raw/guarded stepping。
```

### 8.9 key 3 Locomotion 接入问题与当前修正

早期日志证明 key `3` 一进入 `LOCOMOTION + gait=0` 后，ConvexMPC 内部马上进入对角步态相位：

```text
desired_contact=[0.000, 0.714, 0.714, 0.000]
swing_phase=[1.000, 0.000, 0.000, 1.000]

或

desired_contact=[0.643, 0.000, 0.000, 0.643]
swing_phase=[0.000, 0.833, 0.833, 0.000]
```

这说明问题不是 DDS、IMU、q/dq 主链路整体接错，也不是 ConvexMPC trot 算法本身错误，而是：

```text
实物仍处于 key 2 四脚站立状态
但 key 3 直接让 Locomotion 内部进入 gait=0 的某个对角相位
```

同时日志出现大量：

```text
[OUTPUT-GUARD] q_des_delta_clamped
num_qdes_clamped
num_qdes_delta_clamped
qd_delta_clamped
```

这表示原始 `q_des/qd_des` 与当前站立姿态不连续，OutputGuard 只是把突变目标限住，不能替代 gait phase 和 swing trajectory 的正确接入。

当前修正：

```text
key 3:
  BALANCE_STAND
  -> MIT_FORWARD_ARMING
  -> LOCOMOTION_GAIT4_SHADOW
  -> LOCOMOTION_GAIT4_DIRECT
  -> GAIT0_SHADOW
  -> GAIT0_BLEND
  -> GAIT0_DIRECT
```

bridge 层新增检查 / 状态：

```text
bridge_ready
bridge_reject_code
bridge_stage / phase
alpha_q / alpha_qd / alpha_tau
```

2026-07-03 当前已完成的 key `3` 修正：

```text
1. MIT_FORWARD_ARMING 已从 raw readiness 等待改为时间型 warmup。
   之前卡在 low_kp_kd，是因为 STAND_UP raw 在实物 bridge 场景下没有有效 kp/kd。
   现在按 3 后继续发布 Q-STAND-HOLD，runner->run() 正常跑，约 0.5s 后进入 gait4 shadow。

2. LOCOMOTION_GAIT4_SHADOW 已加入 0.15s raw warmup。
   raw warmup 期间不累计 stable，不把第一帧 raw 全 0 / cmpc_debug unavailable 当长期故障。

3. gait4 standing 的 all-stance 判据已改为 mpc_table_now / swing_phase / swing_time。
   contactStates=0.5 在 standing gait 下是正常连续相位值，不再作为 contact_state_not_ready 的硬门槛。

4. gait4 standing 下 foot_pdes jump 不再作为 hard reject。
   实测 pDes 可保持 [0,0,0]，p.z 约 -0.24m，导致 foot_target_jump 约 0.24m。
   这不是脚端目标真实跳变，而是 standing/all-stance 下 pDes 字段不适合作硬门槛。
   该值仍作为 foot_jump_diagnostic 打印。

5. LOCOMOTION_GAIT4_DIRECT 不再触发 raw warmup。
   readiness 参数已拆成 gait4_allstance_check 与 raw_warmup_seconds，避免 direct 阶段再次命中 locomotion_raw_warmup。

6. 当 --test-gait 4 时，key 3 停留在 LOCOMOTION_GAIT4_DIRECT。
   不再自动切 GAIT0_SHADOW，方便单独验证 gait4 direct 能稳定 5~10s。

7. 当 --test-gait != 4 时，从 gait4 direct 进入 GAIT0_SHADOW 前会 reset ConvexMPC gait phase。
   reset 后 firstSwing=true，swingTimeRemaining=0。
   GAIT0_SHADOW 增加 0.20s gait0_raw_warmup，不累计 stable。
   入口第一轮 runner->run() 后打印 gait0_shadow_entry_diagnostics。
```

当前实测结论：

```text
gait4 standing 接管链路已经证明可走通：
MIT_FORWARD_ARMING
-> LOCOMOTION_GAIT4_SHADOW
-> LOCOMOTION_GAIT4_DIRECT

当前阶段尚未宣称 gait0 已通过。
下一目标是让 GAIT0_SHADOW 先做到 bridge_ready=true，连续稳定 0.5s，
且不触发 foot_target_jump / q_des_jump，再进入 GAIT0_BLEND。
```

报告结论更新为：

```text
OutputGuard 不能替代进入 Locomotion 前的 raw readiness；
gait=4 shadow 用于确认 standing raw，gait=4 direct 用于真正闭环站稳；
gait=0 shadow 用于确认 slow-trot raw，GAIT0_BLEND 用于逐步放开 q/qd/tau。
```

### 8.10 dry-output Illegal instruction 定位

一次 `dry-output` 测试中出现：

```text
Illegal instruction
```

gdb 回溯显示崩溃点并不在 MPC/WBC 控制循环中，而是在等待 valid LowState 阶段：

```text
RunControllerOutput
  -> LegBotDDSStateAdapter::WaitForValidLowState()
  -> throw std::runtime_error(...)
  -> _Unwind_RaiseException
  -> /lib/aarch64-linux-gnu/libgcc_s.so.1
  -> autia1716
  -> SIGILL
```

其中 `autia1716` 是 ARM Pointer Authentication 相关指令。当前 OrangePi CPU/系统组合执行到该系统库指令时触发 `SIGILL`。因此这次问题的性质是：

```text
LowState 等待阶段主动抛 C++ 异常
  +
OrangePi C++ 异常展开路径存在 PAC 指令兼容问题
```

而不是：

```text
ConvexMPC 崩溃
WBC/QP 崩溃
电机使能导致崩溃
```

对应修复已经完成：

```text
WaitForAnyLowState()
WaitForValidLowState()
```

从 `throw std::runtime_error(...)` 改成：

```text
return false + reason string
```

调用侧改成：

```text
打印 [LEGBOT-DDS][ERROR] ...
正常 return
不进入 C++ exception unwinder
```

后续原则：

```text
实物启动、等待、状态判断这类可预期失败路径，优先用返回值；
不要依赖 C++ exception 做流程控制。
```

### 8.11 当前编译结果

本地编译通过：

```bash
cd /home/lushilin/Cheetah-Software-master
cmake --build build --target legbot_real_bridge_dryrun -j4
```

结果：

```text
[100%] Built target legbot_real_bridge_dryrun
```

---

## 九、安全策略

### 9.1 当前安全层定位

安全层回答两个问题：

```text
算法命令是否可以 clip 后发布？
真实反馈是否允许继续发布算法输出？
```

它不负责：

- 用 q-only stepping 替代 MIT/MPC/WBC。
- 用 stand hold 替代算法输出。
- 在未通过 readiness / OutputSafetyGuard 前对算法输出做 alpha blend。
- 把 `kp/kd` 改成另一个控制器。
- 替代 ConvexMPC/WBC 的输出。

例外：key `3` 的 `GAIT0_BLEND` 是 bridge 层接管比例，只作用于已经通过 raw sanity 和 OutputSafetyGuard 的 gait=0 输出：

```text
q_pub   = (1 - alpha_q) * q_hold + alpha_q * q_raw
qd_pub  = alpha_qd * qd_raw
tau_pub = alpha_tau * tau_raw
```

### 9.2 命令侧 OutputSafetyGuard

命令侧 OutputSafetyGuard 只在 `--real-output-guarded` 下启用。它作用于：

```text
FromSpiCommand() 之后
PublishLowCmd() 之前
```

clip 项：

```text
action/mode
q_des absolute range
q_des delta
qd_des delta
tau_ff
Kp/Kd max
```

clip 后继续发布，并打印：

```text
[LEGBOT-DDS][OUTPUT-GUARD] q_des_clamped
[LEGBOT-DDS][OUTPUT-GUARD] q_des_delta_clamped
[LEGBOT-DDS][OUTPUT-GUARD] qd_delta_clamped
[LEGBOT-DDS][OUTPUT-GUARD] tau_clamped
[LEGBOT-DDS][OUTPUT-GUARD] kp_kd_clamped
```

若 `Kp/Kd` 非有限值，或 `max_kp < 1 / max_kd < 0.1`，首轮停止发布算法输出。

`--real-output-raw` 下跳过本节所有命令侧 clip：

```text
q_des raw == q_des pub
tau raw == tau pub
kp/kd raw == kp/kd pub
num_*_clamped == 0
```

raw 模式只用于短时间观察算法原始输出，例如 key `3` 原地 stepping 的第一批周期。它不适合作为常规长时间实物运行模式。

### 9.3 反馈侧 RuntimeSafetyGuard

这些真实反馈错误不应继续出力：

```text
rt/lowstate timeout
tick stale
motor lost
q/dq 非有限值
真实 q 越界
真实 dq 超限
真实 tau_est 超限
电机温度超限
IMU quaternion 非有限值
IMU quaternion norm 异常
gyro 非有限值
roll/pitch 超限
gyro 超限
```

触发后：

```text
停止发布 MIT 算法输出
disable / Passive
exit_reason=runtime_safety_failed:<reason>
正常 return
不执行 prone ramp
不 throw
不 Aborted
```

### 9.4 日志判读

key `2/3` 每 0.2s 打印 raw/pub 对比：

```text
q_des_raw
q_des_pub
tau_raw
tau_pub
kp_raw / kp_pub
kd_raw / kd_pub
num_qdes_clamped
num_qdes_delta_clamped
num_tau_clamped
max_qdes_delta
q_feedback
dq_feedback
tilt
gyro
```

如果 clip 频繁发生，不代表测试成功，而是说明算法输出和实物安全范围不匹配。优先检查：

```text
腿序
零位
joint sign
状态估计
MPC/WBC 参数
```

---

## 十、下一步建议

### 10.1 大方向一：先稳定 BALANCE_STAND

优先用交互模式：

```text
1 q-only 站起
2 MIT BALANCE_STAND direct
观察 5~10 秒
4 回趴退出
```

需要确认：

- FSM 不再报非法跳转。
- `fsm_current=BALANCE_STAND`，`control_mode=3`。
- `MIT-BALANCE-STAND-DIRECT` 阶段 `max_kp/max_kd/q_des_raw/tau_raw` 不再全 0。
- guarded 模式下 `q_des_pub/tau_pub/kp_pub/kd_pub` 由 OutputSafetyGuard 正常输出。
- guarded 模式下 `num_qdes_clamped/num_qdes_delta_clamped/num_tau_clamped` 不应长期高频增长。
- 机器人不明显下沉、抖动、单腿方向错误。
- `dds_q` 与 `q_des` 的方向一致。
- gateway `decode_errors/tx_write_errors` 正常。

### 10.2 大方向二：key 3 原地 slow-trot 安全接管

BALANCE_STAND 稳定后：

```text
3 MIT LOCOMOTION gait4 shadow/direct -> gait0 shadow/blend/direct
x 回到 Q-STAND-HOLD
4 回趴退出
```

guarded 第一轮先只测 gait4 standing 接管：

```text
test_gait = 4
algo_forward_vx = 0.0
cmd_tau_limit = 12.0
cmd_qdes_delta_limit = 0.03
cmd_qd_delta_limit = 0.5
```

验收 LOCOMOTION_GAIT4_SHADOW / DIRECT：

```text
MIT_FORWARD_ARMING 不再卡 low_kp_kd
LOCOMOTION_GAIT4_SHADOW 前 0.15s 只显示 locomotion_raw_warmup
mpc_table_now == [1,1,1,1]
swing_phase ~= [0,0,0,0]
swing_time ~= [0,0,0,0]
desired_vx = 0
RR/RL thigh q_des_raw >= 0.65
contactStates=0.5 不再导致 reject
foot_jump_diagnostic 只打印，不阻止 gait4 standing
tilt < 0.10 rad
连续稳定后才进入 LOCOMOTION_GAIT4_DIRECT
DIRECT 阶段不再出现 locomotion_raw_warmup
DIRECT 阶段稳定 5~10s
tau_pub 不再一直为 0
q_des_pub 接近 q_des_raw 或 guarded 后 raw
没有 HARD-FAULT，没有明显持续 clip
```

gait4 direct 稳定后，再单独测 gait0 shadow：

```text
test_gait = 0
algo_forward_vx = 0.0
```

验收 GAIT0_SHADOW：

```text
从 gait4 direct 切入前打印 reset ConvexMPC gait phase
GAIT0_SHADOW 前 0.20s 显示 gait0_raw_warmup
gait0_shadow_entry_diagnostics 打印 mpc_table_now / swing_phase / swing_time / foot p-pDes diff / 最大 q_des_raw-q_feedback 关节
cmpc_swing_time ~= 0.26s
第一帧 swing_phase 不直接接近 0.8
foot_pdes 与 foot_p 轴向差值 < 0.08m
RR/RL thigh q_des_raw >= 0.55
max_abs_qd_des_raw <= 6.0 rad/s
bridge_ready = true 后才进入 GAIT0_BLEND
连续稳定 0.5s
不触发 foot_target_jump
不触发 q_des_jump
暂时不测前进速度
```

GAIT0_SHADOW 稳定后，才进入 GAIT0_BLEND 验收：

```text
alpha_q / alpha_qd / alpha_tau 平滑从 0 到 1
raw 仍通过 readiness 与 OutputSafetyGuard
没有 HARD-FAULT，没有明显持续 clip
```

raw 第一轮：

```text
--real-output-raw
algo_forward_vx = 0.0
test_gait = 0
tau_ff_scale = 0.0
观察 shadow/blend 阶段 raw/pub 差异，必要时按 x 或断电
```

稳定后再逐步放宽：

```text
cmd_qdes_delta_limit 0.03 -> 0.05
cmd_qd_delta_limit 0.5 -> 1.0
cmd_tau_limit 12.0 -> 更高
```

如果 GAIT0_SHADOW / BLEND 仍有明显前后腿不连续，再继续查：

```text
FootSwingTrajectory p0 是否来自当前 foot position
vWorld_x 估计是否先出现明显负值
nominal foot position / body height / CoM 参数
LegBot 模型质量与足端位置
```

### 10.3 大方向三：校准映射与 sign

如果出现：

- 单腿反向。
- 左右腿对称错误。
- calf 方向异常。
- q_des 合理但实物动作相反。

优先检查：

```text
LegBotJointMap.h
sign array
ModelToDdsJoint / DdsToModelJoint
gateway 中的 joint map 与 bias
```

不要先改 MPC/WBC。

### 10.4 大方向四：继续完善真实状态估计

当前实物链路坚持 `cheater_mode=0`，主状态估计来自 `VectorNavData + SpiData`。后续可以分阶段推进：

1. 检查 IMU roll/pitch 与实物方向是否一致。
2. 补真实 accelerometer，避免长期使用静止重力近似。
3. 检查 `LinearKFPositionVelocityEstimator` 的 `vBody/vWorld` 是否仍有 1 m/s 量级异常。
4. 检查足端运动学、contactEstimate、base height 与 LegBot 实物是否匹配。

### 10.5 大方向五：将 bridge 正式化

当前文件名仍是：

```text
tools/legbot_real_bridge_dryrun.cpp
```

后续建议：

```text
tools/legbot_real_bridge.cpp
```

并接入：

```text
main_helper
RobotType::LEGBOT + real
```

但在 BALANCE_STAND 和低速踏步稳定前，保持独立可执行更安全，调试边界也更清晰。

---

## 附录：当前推荐最小测试顺序

```text
1. 电机通电
2. 启动 dds_to_serial_gateway
3. dry-run 看 lost/tick/q/IMU
4. interactive-control 启动 bridge
5. 按 1：q-only 站起
6. 按 2：MIT BALANCE_STAND direct
7. guarded 模式观察 raw/pub、clip 统计、q_feedback/dq_feedback、tilt/gyro；raw 模式观察 raw 是否合理
8. 第一轮 --test-gait 4，按 3：验证 gait4 shadow -> gait4 direct，稳定 5~10s
9. 按 x：回 Q-STAND-HOLD，再按 4 回趴退出
10. 第二轮 --test-gait 0，按 3：重点观察 GAIT0_SHADOW 是否 ready，不急着测前进速度
```

这条路线最符合当前目标：先确认通信和姿态安全，再逐步把真实输出交给 Cheetah 的 ConvexMPC/WBC 控制链。

---

## 十一、当前进展快照

更新时间：2026-07-03。

当前已经做到：

```text
1. DDS-only bridge 能在实物上订阅 LowState、发布 guarded LowCmd。
2. q-only stand-up / Q-STAND-HOLD / shutdown ramp 已能作为人工安全底座。
3. key 2 已调整为 MIT BALANCE_STAND direct，避免用 Locomotion standing 承担首轮站立接管。
4. key 3 已建立 bridge 层状态机：
   MIT_FORWARD_ARMING
   -> LOCOMOTION_GAIT4_SHADOW
   -> LOCOMOTION_GAIT4_DIRECT
   -> GAIT0_SHADOW
   -> GAIT0_BLEND
   -> GAIT0_DIRECT
5. MIT_FORWARD_ARMING 不再等待 STAND_UP raw kp/kd，因此不再卡 low_kp_kd。
6. gait4 shadow 已解决三个误判：
   - raw 刚切入前 0.15s warmup
   - contactStates=0.5 不再被当成 not all-stance
   - foot_pdes jump 在 gait4 standing 下只做诊断，不做 hard reject
7. gait4 direct 不再二次触发 raw warmup。
8. --test-gait 4 时会停留在 LOCOMOTION_GAIT4_DIRECT，不自动进入 gait0。
9. --test-gait 0 时，进入 GAIT0_SHADOW 前会 reset ConvexMPC gait phase，
   并在 GAIT0_SHADOW 做 0.20s raw warmup 与入口诊断。
```

当前最重要的实测结论：

```text
gait4 standing 接管链路已经基本跑通。
现在不要直接测前进速度。
下一步目标是让 GAIT0_SHADOW 本身先稳定 ready：
bridge_ready=true
连续稳定 0.5s
不触发 foot_target_jump
不触发 q_des_jump
然后再考虑 GAIT0_BLEND。
```

当前推荐测试顺序：

```text
第一轮：
  --test-gait 4
  按 1 站起
  按 3
  验证 LOCOMOTION_GAIT4_DIRECT 稳定 5~10s
  x 回 Q-STAND-HOLD
  4 回趴退出

第二轮：
  --test-gait 0
  按 1 站起
  按 3
  只重点观察 GAIT0_SHADOW
  看 gait0_raw_warmup 后是否 bridge_ready=true
  看 foot_target_jump / q_des_jump 是否还触发
  暂时不加前进速度
```

#ifndef PROJECT_LEGBOT_DDS_ADAPTERS_H
#define PROJECT_LEGBOT_DDS_ADAPTERS_H

#include "SimUtilities/IMUTypes.h"
#include "SimUtilities/SpineBoard.h"
#include "LegBotDDS/LegBotJointMap.h"
#include "LegBotDDS/LegBotSafetySupervisor.h"

#include <Math/orientation_tools.h>

#include "unitree/dds_wrapper/robots/go2/go2_pub.h"
#include "unitree/dds_wrapper/robots/go2/go2_sub.h"

#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

namespace legbot {

using LowCmdPublisher = unitree::robot::go2::publisher::LowCmd;
using LowStateSubscriber = unitree::robot::go2::subscription::LowState;
using LowCmdMsg = unitree_go::msg::dds_::LowCmd_;
using LowStateMsg = unitree_go::msg::dds_::LowState_;

constexpr double kGyroDeadzoneXY = 0.05;
constexpr double kGyroDeadzoneZ = 0.01;

inline double ApplyDeadzone(double value, double deadzone) {
  return std::abs(value) < deadzone ? 0. : value;
}

struct LowStateSnapshot {
  LowStateMsg msg;
  std::array<double, kNumJoints> dds_q{};
  std::array<double, kNumJoints> dds_dq{};
  std::array<double, kNumJoints> dds_tau_est{};
  std::array<double, kNumJoints> motor_temp{};
  std::array<double, kNumJoints> model_q{};
  std::array<double, kNumJoints> model_dq{};
  std::array<double, kNumJoints> model_tau_est{};
  std::array<float, 4> imu_quat_wxyz{{1.f, 0.f, 0.f, 0.f}};
  std::array<double, 3> gyro{{0., 0., 0.}};
  uint32_t tick = 0;
};

struct RuntimeSafetyResult {
  bool ok = true;
  std::string reason;
  int joint = -1;
  double value = 0.;
};

class LegBotDDSStateAdapter {
 public:
  explicit LegBotDDSStateAdapter(double lowstate_timeout_s)
      : lowstate_timeout_s_(lowstate_timeout_s) {}

  void SetSubscriber(const std::shared_ptr<LowStateSubscriber>& sub) {
    sub_ = sub;
    if (sub_) {
      sub_->set_timeout_ms(
          static_cast<uint32_t>(std::ceil(lowstate_timeout_s_ * 1000.)));
    }
  }

  bool IsTimeout() const { return !sub_ || sub_->isTimeout(); }

  LowStateSnapshot Snapshot() const {
    if (!sub_) throw std::runtime_error("LowState subscriber is not set");
    std::lock_guard<std::mutex> lock(sub_->mutex_);
    LowStateSnapshot snapshot;
    snapshot.msg = sub_->msg_;
    FillSnapshotFromMsg(snapshot.msg, &snapshot);
    return snapshot;
  }

  bool HasLostMotor(const LowStateMsg& msg, std::string* lost_joint) const {
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      if (msg.motor_state()[i].lost() != 0u) {
        if (lost_joint) *lost_joint = JointNameDds(static_cast<int>(i));
        return true;
      }
    }
    return false;
  }

  std::string LostFlagsText(const LowStateMsg& msg) const {
    std::ostringstream oss;
    bool any = false;
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      const auto lost = msg.motor_state()[i].lost();
      if (lost != 0u) {
        if (any) oss << ",";
        oss << JointNameDds(static_cast<int>(i)) << ":"
            << static_cast<unsigned>(lost);
        any = true;
      }
    }
    return any ? oss.str() : std::string("none");
  }

  bool WaitForAnyLowState(const BridgeArgs& args,
                          std::string* reason = nullptr) const {
    const auto deadline =
        Clock::now() + std::chrono::duration<double>(args.wait_lowstate_s);
    while (Clock::now() < deadline) {
      if (!IsTimeout()) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (reason) *reason = "no rt/lowstate received";
    return false;
  }

  bool WaitForValidLowState(const BridgeArgs& args,
                            std::string* reason = nullptr) const {
    const auto deadline =
        Clock::now() + std::chrono::duration<double>(args.wait_lowstate_s);
    while (Clock::now() < deadline) {
      if (!IsTimeout()) {
        const auto snap = Snapshot();
        if (!HasLostMotor(snap.msg, nullptr)) return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (reason) {
      *reason = IsTimeout() ? "no rt/lowstate received"
                            : "motor feedback is still lost";
    }
    return false;
  }

  RuntimeSafetyResult CheckRuntimeSafety(
      const LowStateSnapshot& snapshot,
      const std::array<double, kNumJoints>* model_q_target,
      bool enable_qerr_check, const BridgeArgs& args, LoopStats* stats) const {
    std::string lost_joint;
    if (HasLostMotor(snapshot.msg, &lost_joint)) {
      return {false, "motor_lost:" + lost_joint, -1, 0.};
    }

    for (double g : snapshot.gyro) {
      if (!std::isfinite(g)) {
        return {false, "gyro_nonfinite", -1, g};
      }
      if (std::abs(g) > args.max_gyro_rad_s) {
        return {false, "max_gyro_rad_s", -1, std::abs(g)};
      }
    }

    if (enable_qerr_check && model_q_target && args.max_q_error > 0.) {
      int qerr_joint = 0;
      const double qerr =
          MaxAbsError(snapshot.model_q, *model_q_target, &qerr_joint);
      if (stats) stats->max_qerr = std::max(stats->max_qerr, qerr);
      if (qerr > args.max_q_error) {
        return {false,
                std::string("max_q_error:") +
                    JointNameDds(ModelToDdsJoint(qerr_joint)),
                qerr_joint, qerr};
      }
    }

    const double tilt = Tilt(snapshot);
    if (stats) stats->max_tilt = std::max(stats->max_tilt, tilt);
    if (tilt > args.max_tilt_rad) {
      return {false, "max_tilt", -1, tilt};
    }

    return {};
  }

  double Tilt(const LowStateSnapshot& snapshot) const {
    const auto rpy = QuatWxyzToRpy(snapshot.imu_quat_wxyz);
    return std::max(std::abs(rpy[0]), std::abs(rpy[1]));
  }

  void FillCheetahData(const LowStateSnapshot& snapshot, SpiData* spi_data,
                       VectorNavData* vector_nav_data) const {
    if (!spi_data || !vector_nav_data) {
      throw std::runtime_error("null Cheetah data pointer");
    }
    for (int leg = 0; leg < static_cast<int>(kNumLegs); ++leg) {
      spi_data->q_abad[leg] =
          static_cast<float>(snapshot.model_q[ModelJointIndex(leg, 0)]);
      spi_data->q_hip[leg] =
          static_cast<float>(snapshot.model_q[ModelJointIndex(leg, 1)]);
      spi_data->q_knee[leg] =
          static_cast<float>(snapshot.model_q[ModelJointIndex(leg, 2)]);
      spi_data->qd_abad[leg] =
          static_cast<float>(snapshot.model_dq[ModelJointIndex(leg, 0)]);
      spi_data->qd_hip[leg] =
          static_cast<float>(snapshot.model_dq[ModelJointIndex(leg, 1)]);
      spi_data->qd_knee[leg] =
          static_cast<float>(snapshot.model_dq[ModelJointIndex(leg, 2)]);
      spi_data->flags[leg] = 0;
    }
    spi_data->spi_driver_status = 0;

    // Current LegBot bring-up does not trust IMU yaw. Keep roll/pitch, force
    // yaw to zero before feeding Cheetah's VectorNav estimator.
    const auto rpy = QuatWxyzToRpy(snapshot.imu_quat_wxyz);
    Vec3<double> rpy_zero_yaw;
    rpy_zero_yaw << rpy[0], rpy[1], 0.;
    const auto q_wxyz = ori::rpyToQuat(rpy_zero_yaw);

    // Cheetah's VectorNavOrientationEstimator expects raw VectorNav order xyzw
    // and then converts xyzw -> internal wxyz, so feed it xyzw here.
    vector_nav_data->quat << static_cast<float>(q_wxyz[1]),
        static_cast<float>(q_wxyz[2]), static_cast<float>(q_wxyz[3]),
        static_cast<float>(q_wxyz[0]);
    vector_nav_data->gyro << static_cast<float>(snapshot.gyro[0]),
        static_cast<float>(snapshot.gyro[1]), 0.f;
    vector_nav_data->accelerometer << 0.f, 0.f, 9.81f;
  }

 private:
  void FillSnapshotFromMsg(const LowStateMsg& msg,
                           LowStateSnapshot* snapshot) const {
    snapshot->tick = msg.tick();
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      snapshot->dds_q[i] = static_cast<double>(msg.motor_state()[i].q());
      snapshot->dds_dq[i] = static_cast<double>(msg.motor_state()[i].dq());
      snapshot->dds_tau_est[i] =
          static_cast<double>(msg.motor_state()[i].tau_est());
      snapshot->motor_temp[i] =
          static_cast<double>(msg.motor_state()[i].temperature());
    }
    snapshot->model_q = DdsQToModelQ(snapshot->dds_q);
    snapshot->model_dq = DdsDqToModelDq(snapshot->dds_dq);
    snapshot->model_tau_est = DdsQToModelQ(snapshot->dds_tau_est);
    snapshot->imu_quat_wxyz = msg.imu_state().quaternion();
    const auto gyro = msg.imu_state().gyroscope();
    snapshot->gyro = {
        {ApplyDeadzone(static_cast<double>(gyro[0]), kGyroDeadzoneXY),
         ApplyDeadzone(static_cast<double>(gyro[1]), kGyroDeadzoneXY),
         ApplyDeadzone(static_cast<double>(gyro[2]), kGyroDeadzoneZ)}};
  }

  double lowstate_timeout_s_;
  std::shared_ptr<LowStateSubscriber> sub_;
};

struct DdsCommandArrays {
  std::array<double, kNumJoints> q{};
  std::array<double, kNumJoints> dq{};
  std::array<double, kNumJoints> kp{};
  std::array<double, kNumJoints> kd{};
  std::array<double, kNumJoints> tau{};
  std::array<uint8_t, kNumJoints> mode{};
};

inline double GatewayMotorTauScaleForDdsJoint(std::size_t dds_joint) {
  // dds_to_serial_gateway converts q/dq between model and motor space, but its
  // current tau path is 1:1. Pre-convert tau here using the same sign/gear
  // convention so the gateway can remain unchanged.
  static const std::array<double, kNumJoints> scale = {
      1.0, 1.0, 1.0,    // FR hip/thigh/calf
      1.0, -1.0, -1.0,  // FL hip/thigh/calf
      1.0, 1.0, 1.0,    // RR hip/thigh/calf
      1.0, -1.0, -1.0,  // RL hip/thigh/calf
  };
  return scale.at(dds_joint);
}

struct OutputGuardStats {
  DdsCommandArrays raw;
  DdsCommandArrays pub;
  int num_action_clamped = 0;
  int num_qdes_clamped = 0;
  int num_qdes_delta_clamped = 0;
  int num_qd_delta_clamped = 0;
  int num_tau_clamped = 0;
  int num_kp_kd_clamped = 0;
  double max_abs_tau_raw = 0.;
  double max_abs_tau_pub = 0.;
  double max_qdes_delta = 0.;
  std::string fault_reason;
};

inline bool IsFiniteArray(const std::array<double, kNumJoints>& values) {
  for (const double v : values) {
    if (!std::isfinite(v)) return false;
  }
  return true;
}

inline const std::array<double, kNumJoints>& StandQDds() {
  static const std::array<double, kNumJoints> q = {
      -0.0, 0.9, -1.8, 0.0, 0.9, -1.8,
      -0.0, 0.9, -1.8, 0.0, 0.9, -1.8,
  };
  return q;
}

inline const std::array<double, kNumJoints>& DownQDds() {
  static const std::array<double, kNumJoints> q = {
      -0.02, 1.08, -2.64, 0.03, 1.08, -2.64,
      -0.05, 1.08, -2.64, 0.06, 1.08, -2.64,
  };
  return q;
}

inline double SmoothStep(double x) {
  x = Clamp(x, 0., 1.);
  return x * x * (3. - 2. * x);
}

inline std::pair<double, double> CmdQDesLimitForModelJoint(
    int model_joint, const BridgeArgs& args) {
  const int joint = model_joint % static_cast<int>(kNumJointsPerLeg);
  if (joint == 0) return {-args.cmd_hip_abs_limit, args.cmd_hip_abs_limit};
  if (joint == 1) return {args.cmd_thigh_min, args.cmd_thigh_max};
  return {args.cmd_calf_min, args.cmd_calf_max};
}

inline std::pair<double, double> FeedbackQLimitForModelJoint(
    int model_joint, const BridgeArgs& args) {
  const int joint = model_joint % static_cast<int>(kNumJointsPerLeg);
  if (joint == 0) return {-args.fb_hip_abs_limit, args.fb_hip_abs_limit};
  if (joint == 1) return {args.fb_thigh_min, args.fb_thigh_max};
  return {args.fb_calf_min, args.fb_calf_max};
}

inline RuntimeSafetyResult CheckFeedbackJointLimits(
    const LowStateSnapshot& snapshot, const BridgeArgs& args) {
  if (!IsFiniteArray(snapshot.model_q) || !IsFiniteArray(snapshot.model_dq) ||
      !IsFiniteArray(snapshot.model_tau_est) ||
      !IsFiniteArray(snapshot.motor_temp)) {
    return {false, "lowstate_q_or_dq_nonfinite", -1, 0.};
  }
  for (std::size_t model = 0; model < kNumJoints; ++model) {
    const auto limit = FeedbackQLimitForModelJoint(static_cast<int>(model), args);
    const double q = snapshot.model_q[model];
    const int dds = ModelToDdsJoint(static_cast<int>(model));
    if (q < limit.first || q > limit.second) {
      return {false,
              std::string("runtime_joint_limit:") +
                  JointNameDds(dds),
              dds, q};
    }
    const double dq = snapshot.model_dq[model];
    if (args.fb_qd_limit > 0. && std::abs(dq) > args.fb_qd_limit) {
      return {false, std::string("runtime_qd_limit:") + JointNameDds(dds),
              dds, std::abs(dq)};
    }
    const double tau = snapshot.model_tau_est[model];
    if (args.fb_tau_limit > 0. && std::abs(tau) > args.fb_tau_limit) {
      return {false, std::string("runtime_tau_limit:") + JointNameDds(dds),
              dds, std::abs(tau)};
    }
    const double temp = snapshot.motor_temp[static_cast<std::size_t>(dds)];
    if (temp >= args.fb_temp_limit) {
      return {false, std::string("runtime_temp_limit:") + JointNameDds(dds),
              dds, temp};
    }
    if (temp >= args.fb_temp_warning) {
      std::cout << "[LEGBOT-DDS][RUNTIME-SAFETY][WARN] temp_warning joint="
                << JointNameDds(dds) << " index=" << dds
                << " temp=" << temp << " warning=" << args.fb_temp_warning
                << " limit=" << args.fb_temp_limit << "\n";
    }
  }
  return {};
}

inline bool ApplyOutputGuard(
    const DdsCommandArrays& raw, const BridgeArgs& args,
    const std::array<double, kNumJoints>* previous_dds_q_des,
    const std::array<double, kNumJoints>* previous_dds_qd_des,
    OutputGuardStats* guard) {
  if (!guard) return false;
  guard->raw = raw;
  guard->pub = raw;
  guard->num_action_clamped = 0;
  guard->num_qdes_clamped = 0;
  guard->num_qdes_delta_clamped = 0;
  guard->num_qd_delta_clamped = 0;
  guard->num_tau_clamped = 0;
  guard->num_kp_kd_clamped = 0;
  guard->max_abs_tau_raw = 0.;
  guard->max_abs_tau_pub = 0.;
  guard->max_qdes_delta = 0.;
  guard->fault_reason.clear();

  for (std::size_t i = 0; i < kNumJoints; ++i) {
    if (!std::isfinite(raw.q[i]) || !std::isfinite(raw.dq[i]) ||
        !std::isfinite(raw.tau[i]) || !std::isfinite(raw.kp[i]) ||
        !std::isfinite(raw.kd[i])) {
      guard->fault_reason =
          std::string("command_nonfinite:") + JointNameDds(static_cast<int>(i));
      return false;
    }
  }

  double max_kp = 0.;
  double max_kd = 0.;
  for (std::size_t dds = 0; dds < kNumJoints; ++dds) {
    if (static_cast<double>(guard->pub.mode[dds]) > args.cmd_action_limit) {
      const auto clamped_mode =
          static_cast<uint8_t>(std::min(args.cmd_action_limit, 255.0));
      std::cout << "[LEGBOT-DDS][OUTPUT-GUARD] action_clamped joint="
                << JointNameDds(static_cast<int>(dds)) << " index=" << dds
                << " raw=" << static_cast<unsigned>(guard->pub.mode[dds])
                << " clamped=" << static_cast<unsigned>(clamped_mode)
                << " limit=" << args.cmd_action_limit << "\n";
      guard->pub.mode[dds] = clamped_mode;
      ++guard->num_action_clamped;
    }

    const int model = DdsToModelJoint(static_cast<int>(dds));
    const auto q_limit = CmdQDesLimitForModelJoint(model, args);
    const double raw_q = guard->pub.q[dds];
    const double range_clamped_q = Clamp(raw_q, q_limit.first, q_limit.second);
    if (range_clamped_q != raw_q) {
      std::cout << "[LEGBOT-DDS][OUTPUT-GUARD] q_des_clamped joint="
                << JointNameDds(static_cast<int>(dds)) << " index=" << dds
                << " raw=" << raw_q << " clamped=" << range_clamped_q
                << " limit=[" << q_limit.first << "," << q_limit.second
                << "]\n";
      guard->pub.q[dds] = range_clamped_q;
      ++guard->num_qdes_clamped;
    }

    if (previous_dds_q_des && args.cmd_qdes_delta_limit > 0.) {
      const double prev = (*previous_dds_q_des)[dds];
      const double delta = guard->pub.q[dds] - prev;
      guard->max_qdes_delta =
          std::max(guard->max_qdes_delta, std::abs(delta));
      if (std::abs(delta) > args.cmd_qdes_delta_limit) {
        const double clamped =
            prev + Clamp(delta, -args.cmd_qdes_delta_limit,
                         args.cmd_qdes_delta_limit);
        std::cout << "[LEGBOT-DDS][OUTPUT-GUARD] q_des_delta_clamped joint="
                  << JointNameDds(static_cast<int>(dds)) << " index=" << dds
                  << " raw=" << guard->pub.q[dds] << " clamped=" << clamped
                  << " limit=+/-" << args.cmd_qdes_delta_limit << "\n";
        guard->pub.q[dds] = clamped;
        ++guard->num_qdes_clamped;
        ++guard->num_qdes_delta_clamped;
      }
    }

    if (previous_dds_qd_des && args.cmd_qd_delta_limit > 0.) {
      const double prev = (*previous_dds_qd_des)[dds];
      const double delta = guard->pub.dq[dds] - prev;
      if (std::abs(delta) > args.cmd_qd_delta_limit) {
        const double clamped =
            prev + Clamp(delta, -args.cmd_qd_delta_limit,
                         args.cmd_qd_delta_limit);
        std::cout << "[LEGBOT-DDS][OUTPUT-GUARD] qd_delta_clamped joint="
                  << JointNameDds(static_cast<int>(dds)) << " index=" << dds
                  << " raw=" << guard->pub.dq[dds] << " clamped=" << clamped
                  << " limit=+/-" << args.cmd_qd_delta_limit << "\n";
        guard->pub.dq[dds] = clamped;
        ++guard->num_qd_delta_clamped;
      }
    }

    guard->max_abs_tau_raw =
        std::max(guard->max_abs_tau_raw, std::abs(raw.tau[dds]));
    if (args.cmd_tau_limit > 0.) {
      const double tau = guard->pub.tau[dds];
      const double clamped_tau =
          Clamp(tau, -args.cmd_tau_limit, args.cmd_tau_limit);
      if (clamped_tau != tau) {
        std::cout << "[LEGBOT-DDS][OUTPUT-GUARD] tau_clamped joint="
                  << JointNameDds(static_cast<int>(dds)) << " index=" << dds
                  << " raw=" << tau << " clamped=" << clamped_tau
                  << " limit=+/-" << args.cmd_tau_limit << "\n";
        guard->pub.tau[dds] = clamped_tau;
        ++guard->num_tau_clamped;
      }
    }
    guard->max_abs_tau_pub =
        std::max(guard->max_abs_tau_pub, std::abs(guard->pub.tau[dds]));

    const double kp = guard->pub.kp[dds];
    const double kd = guard->pub.kd[dds];
    const double kp_clamped = Clamp(kp, args.cmd_kp_min, args.cmd_kp_max);
    const double kd_clamped = Clamp(kd, args.cmd_kd_min, args.cmd_kd_max);
    if (kp_clamped != kp || kd_clamped != kd) {
      std::cout << "[LEGBOT-DDS][OUTPUT-GUARD] kp_kd_clamped joint="
                << JointNameDds(static_cast<int>(dds)) << " index=" << dds
                << " kp_raw=" << kp << " kp_clamped=" << kp_clamped
                << " kd_raw=" << kd << " kd_clamped=" << kd_clamped
                << " kp_limit=[" << args.cmd_kp_min << ","
                << args.cmd_kp_max << "] kd_limit=[" << args.cmd_kd_min
                << "," << args.cmd_kd_max << "]\n";
      guard->pub.kp[dds] = kp_clamped;
      guard->pub.kd[dds] = kd_clamped;
      ++guard->num_kp_kd_clamped;
    }
    max_kp = std::max(max_kp, guard->pub.kp[dds]);
    max_kd = std::max(max_kd, guard->pub.kd[dds]);
  }
  if (max_kp < 1.0 || max_kd < 0.1) {
    std::cout << "[LEGBOT-DDS][OUTPUT-GUARD][WARN] low_kp_kd max_kp="
              << max_kp << " max_kd=" << max_kd
              << " threshold_kp=1 threshold_kd=0.1\n";
    guard->fault_reason = "low_kp_kd";
    return false;
  }
  return true;
}

class LegBotDDSCommandAdapter {
 public:
  DdsCommandArrays FromSpiCommand(const SpiCommand& spi_command) const {
    std::array<double, kNumJoints> model_q{};
    std::array<double, kNumJoints> model_dq{};
    std::array<double, kNumJoints> model_kp{};
    std::array<double, kNumJoints> model_kd{};
    std::array<double, kNumJoints> model_tau{};
    std::array<uint8_t, kNumJoints> model_mode{};

    for (int leg = 0; leg < static_cast<int>(kNumLegs); ++leg) {
      model_q[ModelJointIndex(leg, 0)] = spi_command.q_des_abad[leg];
      model_q[ModelJointIndex(leg, 1)] = spi_command.q_des_hip[leg];
      model_q[ModelJointIndex(leg, 2)] = spi_command.q_des_knee[leg];
      model_dq[ModelJointIndex(leg, 0)] = spi_command.qd_des_abad[leg];
      model_dq[ModelJointIndex(leg, 1)] = spi_command.qd_des_hip[leg];
      model_dq[ModelJointIndex(leg, 2)] = spi_command.qd_des_knee[leg];
      model_kp[ModelJointIndex(leg, 0)] = spi_command.kp_abad[leg];
      model_kp[ModelJointIndex(leg, 1)] = spi_command.kp_hip[leg];
      model_kp[ModelJointIndex(leg, 2)] = spi_command.kp_knee[leg];
      model_kd[ModelJointIndex(leg, 0)] = spi_command.kd_abad[leg];
      model_kd[ModelJointIndex(leg, 1)] = spi_command.kd_hip[leg];
      model_kd[ModelJointIndex(leg, 2)] = spi_command.kd_knee[leg];
      model_tau[ModelJointIndex(leg, 0)] = spi_command.tau_abad_ff[leg];
      model_tau[ModelJointIndex(leg, 1)] = spi_command.tau_hip_ff[leg];
      model_tau[ModelJointIndex(leg, 2)] = spi_command.tau_knee_ff[leg];
      for (int joint = 0; joint < static_cast<int>(kNumJointsPerLeg); ++joint) {
        model_mode[ModelJointIndex(leg, joint)] =
            spi_command.flags[leg] ? uint8_t{1} : uint8_t{0};
      }
    }

    DdsCommandArrays out;
    out.q = ModelQToDdsQ(model_q);
    out.dq = ModelQToDdsQ(model_dq);
    out.tau = ModelTauToDdsTau(model_tau);
    for (std::size_t model = 0; model < kNumJoints; ++model) {
      const std::size_t dds = static_cast<std::size_t>(ModelToDdsJoint(model));
      out.mode[dds] = model_mode[model];
      out.kp[dds] = model_kp[model];
      out.kd[dds] = model_kd[model];
      out.tau[dds] *= GatewayMotorTauScaleForDdsJoint(dds);
    }
    return out;
  }

  DdsCommandArrays MakeQOnlyCommand(const std::array<double, kNumJoints>& dds_q,
                                    double kp, double kd) const {
    DdsCommandArrays out;
    out.q = dds_q;
    out.dq.fill(0.);
    out.kp.fill(kp);
    out.kd.fill(kd);
    out.tau.fill(0.);
    out.mode.fill(1);
    return out;
  }

  void ApplyTauScale(double scale, DdsCommandArrays* arrays) const {
    if (!arrays) throw std::runtime_error("null DdsCommandArrays pointer");
    for (double& tau : arrays->tau) tau *= scale;
  }

  void FillLowCmd(const DdsCommandArrays& arrays, LowCmdMsg* cmd) const {
    if (!cmd) throw std::runtime_error("null LowCmd pointer");
    cmd->head() = {0xFE, 0xEF};
    cmd->level_flag() = 0xFF;
    cmd->gpio() = 0;
    for (std::size_t i = 0; i < cmd->motor_cmd().size(); ++i) {
      auto& motor = cmd->motor_cmd()[i];
      if (i < kNumJoints) {
        motor.mode(arrays.mode[i]);
        motor.q(static_cast<float>(arrays.q[i]));
        motor.dq(static_cast<float>(arrays.dq[i]));
        motor.kp(static_cast<float>(arrays.kp[i]));
        motor.kd(static_cast<float>(arrays.kd[i]));
        motor.tau(static_cast<float>(arrays.tau[i]));
      } else {
        motor.mode(0);
        motor.q(0.f);
        motor.dq(0.f);
        motor.kp(0.f);
        motor.kd(0.f);
        motor.tau(0.f);
      }
    }
  }

  bool PublishLowCmd(LowCmdPublisher* pub, const DdsCommandArrays& arrays,
                     LoopStats* stats, const Clock::time_point& stats_t0) const {
    if (!pub) throw std::runtime_error("LowCmd publisher is not set");
    const auto deadline = Clock::now() + std::chrono::milliseconds(50);
    while (Clock::now() < deadline) {
      if (pub->trylock()) {
        FillLowCmd(arrays, &pub->msg_);
        pub->unlockAndPublish();
        if (stats) {
          ++stats->publish_count;
          stats->publish_times_s.push_back(SecondsSince(stats_t0, Clock::now()));
        }
        return true;
      }
      std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
    return false;
  }

  void DisableBurst(LowCmdPublisher* pub, const BridgeArgs& args,
                    LoopStats* stats, const Clock::time_point& stats_t0) const {
    if (!pub) return;
    DdsCommandArrays disable;
    disable.mode.fill(0);
    const int count = std::max(
        3, static_cast<int>(std::ceil(
               std::min(0.25, 5.0 / std::max(args.cmd_hz, 1.0)) *
               std::max(args.cmd_hz, 1.0))));
    for (int i = 0; i < count; ++i) {
      PublishLowCmd(pub, disable, stats, stats_t0);
      std::this_thread::sleep_for(
          std::chrono::duration<double>(1. / std::max(args.cmd_hz, 1.)));
    }
  }
};

inline std::array<double, kNumJoints> DdsCommandModelQ(
    const DdsCommandArrays& command) {
  return DdsQToModelQ(command.q);
}

inline std::array<double, kNumJoints> DdsCommandModelDq(
    const DdsCommandArrays& command) {
  return DdsDqToModelDq(command.dq);
}

inline std::array<double, kNumJoints> DdsCommandModelTau(
    const DdsCommandArrays& command) {
  return DdsQToModelQ(command.tau);
}

}  // namespace legbot

#endif  // PROJECT_LEGBOT_DDS_ADAPTERS_H

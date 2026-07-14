#include "MIT_Controller.hpp"
#include "RobotRunner.h"
#include "LegBotDDS/LegBotDDSAdapters.h"

#include "unitree/robot/channel/channel_factory.hpp"

#include <Configuration.h>
#include <ControlParameters/RobotParameters.h>
#include <Math/orientation_tools.h>
#include <SimUtilities/GamepadCommand.h>
#include <SimUtilities/VisualizationData.h>
#include <Utilities/PeriodicTask.h>

#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

namespace {

std::atomic<bool> g_running{true};

constexpr const char* kLowStateTopic = "rt/lowstate";
constexpr const char* kLowCmdTopic = "rt/lowcmd";
constexpr double kPassiveControlMode = 0.0;
constexpr double kStandUpControlMode = 1.0;
constexpr double kBalanceStandControlMode = 3.0;
constexpr double kLocomotionControlMode = 4.0;
constexpr double kLegBotNominalHeight = 0.260;
constexpr double kReadyMaxAbsQErrorHip = 0.30;
constexpr double kReadyMaxAbsQErrorThigh = 0.35;
constexpr double kReadyMaxAbsQErrorCalf = 0.45;
constexpr double kGait0ShadowMaxAbsQErrorHip = 0.25;
constexpr double kGait0ShadowMaxAbsQErrorThigh = 0.35;
constexpr double kGait0ShadowMaxAbsQErrorCalf = 0.45;
constexpr double kReadyMaxAbsQdDes = 4.0;
constexpr double kPreTrotRearThighMinRaw = 0.65;
constexpr double kGaitEntryMaxAbsQdDes = 6.0;
constexpr double kGaitEntryRearThighMinRaw = 0.55;
constexpr double kKey3ShadowStableSeconds = 0.5;
constexpr double kKey3ShadowMaxWaitSeconds = 5.0;
constexpr double kKey3Gait4DirectSeconds = 3.0;
constexpr double kKey3BlendQSeconds = 3.0;
constexpr double kKey3BlendQdSeconds = 3.0;
constexpr double kKey3BlendTauSeconds = 3.0;
constexpr double kGait0BlendMaxAlphaStep = 0.008;
constexpr double kGait0BlendMaxQStep = 0.02;
constexpr double kGait0BlendTauAlphaScale = 0.3;
constexpr double kGait0BlendQdAlpha = 1.0;
constexpr double kGait0BlendEntryEnvelopeSoftHoldSeconds = 0.5;
constexpr double kGait0BlendHipAlphaScale = 0.55;
constexpr double kGait0BlendThighAlphaScale = 1.15;
constexpr double kGait0BlendCalfAlphaScale = 1.25;
constexpr double kGait0BlendHipMaxQStep = 0.012;
constexpr double kGait0BlendThighMaxQStep = 0.024;
constexpr double kGait0BlendCalfMaxQStep = 0.026;
constexpr double kGait0BlendHipTauAlphaScale = 0.15;
constexpr double kGait0BlendThighTauAlphaScale = 0.35;
constexpr double kGait0BlendCalfTauAlphaScale = 0.30;
constexpr double kGait0BlendStanceHipKpMin = 28.0;
constexpr double kGait0BlendStanceThighKpMin = 38.0;
constexpr double kGait0BlendStanceCalfKpMin = 38.0;
constexpr double kGait0BlendStanceHipKdMin = 2.2;
constexpr double kGait0BlendStanceThighKdMin = 2.5;
constexpr double kGait0BlendStanceCalfKdMin = 2.5;
constexpr double kKey3TiltMaxRad = 0.10;
constexpr double kFootTargetJumpWarnLimit = 0.08;
constexpr double kFootTargetJumpHardLimitDefault = 0.08;
constexpr double kFootTargetJumpHardLimitGait0 = 0.20;
constexpr double kWbcForceWarnMinFz = 100.0;
constexpr double kWbcForceHardMinFzGait0Direct = 70.0;
constexpr double kKey3ArmingForceShadowSeconds = 2.0;
constexpr double kKey3RawWarmupSeconds = 0.15;
constexpr double kGait0ShadowMinComputeOnlySeconds = 2.0;
constexpr double kGait0EntryReferenceSettleSeconds = 0.3;
constexpr double kGait0SwingPhaseBoundaryMargin = 0.06;
constexpr double kGait0RawStableQStepLimit = 0.08;
constexpr double kGait0RawStableMaxAbsQd = 5.0;
constexpr double kGait0ShadowMaxAbsTau = 10.0;
constexpr double kGait0RawVsHoldTakeoverHip = 0.20;
constexpr double kGait0RawVsHoldTakeoverThigh = 0.28;
constexpr double kGait0RawVsHoldTakeoverCalf = 0.35;
constexpr int kGait0RawStableCountRequired = 12;
constexpr int kGait0ShadowMinMpcTableTransitions = 2;
constexpr int kGait0BlendMonitorClampFrames = 3;
constexpr double kGait0BlendMonitorMaxAbsDqFeedback = 3.0;
constexpr double kGait0BlendMonitorMaxTilt = 0.12;
constexpr double kKey3Gait4ToGait0MaxAbsQError = 0.15;
constexpr double kKey3Gait4ToGait0MaxTilt = 0.05;
constexpr double kKey3Gait4ToGait0MaxAbsDqFeedback = 0.5;

enum class ControllerPhase {
  kFsmWarmup,
  kBalanceStand,
  kControllerStand,
  kLocomotionTest,
};

enum class BridgeRejectCode {
  kOk = 0,
  kQDesNotFinite = 1,
  kQDesOutOfRange = 2,
  kQDesJump = 3,
  kQdDesTooLarge = 4,
  kRearThighTooLow = 5,
  kPreTrotTimeout = 6,
  kContactStateNotReady = 7,
  kSwingPhaseNotReady = 8,
  kFootTargetJump = 9,
  kTiltTooLarge = 10,
  kWbcForceTooLow = 11,
};

enum class Key3Phase {
  kNone,
  kMitForwardArming,
  kLocomotionGait4Shadow,
  kLocomotionGait4Direct,
  kGait0Shadow,
  kGait0Blend,
  kGait0Direct,
};

void OnSignal(int) { g_running = false; }

bool RealOutputEnabled(const legbot::BridgeArgs& args) {
  return args.real_output_guarded || args.real_output_raw;
}

bool IsLegBotStationaryTrot(double gait) {
  return std::abs(gait) < 1.e-6;
}

std::array<double, legbot::kNumJoints> ModelQTargetFromDdsCommand(
    const legbot::DdsCommandArrays& command) {
  return legbot::DdsQToModelQ(command.q);
}

void FillCheaterFromLowState(const legbot::LowStateSnapshot& snapshot,
                             CheaterState<double>* cheater_state) {
  const auto rpy_raw = legbot::QuatWxyzToRpy(snapshot.imu_quat_wxyz);
  Vec3<double> rpy_zero_yaw;
  rpy_zero_yaw << rpy_raw[0], rpy_raw[1], 0.;
  cheater_state->orientation = ori::rpyToQuat(rpy_zero_yaw);
  cheater_state->position << 0., 0., kLegBotNominalHeight;
  cheater_state->omegaBody << snapshot.gyro[0], snapshot.gyro[1],
      snapshot.gyro[2];
  cheater_state->vBody.setZero();
  cheater_state->acceleration.setZero();
}

void PrintLowState(const legbot::LowStateSnapshot& snapshot,
                   const legbot::LegBotDDSStateAdapter& state_adapter,
                   const legbot::LoopStats& stats, double elapsed_s) {
  const auto rpy = legbot::QuatWxyzToRpy(snapshot.imu_quat_wxyz);
  std::cout << "[LEGBOT-DDS][DRY-RUN] t=" << std::fixed
            << std::setprecision(2) << elapsed_s << " tick=" << snapshot.tick
            << " lowstate_count=" << stats.lowstate_count
            << " lost=" << state_adapter.LostFlagsText(snapshot.msg)
            << " tilt=" << std::setprecision(5)
            << state_adapter.Tilt(snapshot) << "\n"
            << "  dds_q=" << legbot::ArrayText(snapshot.dds_q, 4) << "\n"
            << "  model_q=" << legbot::ArrayText(snapshot.model_q, 4) << "\n"
            << "  dds_dq=" << legbot::ArrayText(snapshot.dds_dq, 4) << "\n"
            << "  model_dq=" << legbot::ArrayText(snapshot.model_dq, 4) << "\n"
            << "  imu_quat_wxyz="
            << legbot::ArrayText(snapshot.imu_quat_wxyz, 5)
            << " imu_rpy=" << legbot::ArrayText(rpy, 5)
            << " gyro=" << legbot::ArrayText(snapshot.gyro, 5) << "\n";
}

void PrintCommandSummary(const legbot::DdsCommandArrays& command,
                         double elapsed_s, const char* tag = "DRY-OUTPUT") {
  double max_abs_tau = 0.;
  double max_kp = 0.;
  double max_kd = 0.;
  for (std::size_t i = 0; i < legbot::kNumJoints; ++i) {
    max_abs_tau = std::max(max_abs_tau, std::abs(command.tau[i]));
    max_kp = std::max(max_kp, command.kp[i]);
    max_kd = std::max(max_kd, command.kd[i]);
  }
  std::cout << "[LEGBOT-DDS][" << tag << "] t=" << std::fixed
            << std::setprecision(2) << elapsed_s
            << " max_abs_tau=" << std::setprecision(4) << max_abs_tau
            << " max_kp=" << max_kp << " max_kd=" << max_kd << "\n"
            << "  dds_q_des=" << legbot::ArrayText(command.q, 4) << "\n"
            << "  dds_tau_ff=" << legbot::ArrayText(command.tau, 4) << "\n";
}

void ApplyControllerCommand(ControllerPhase phase, double gait, double vx,
                            double vy, double yaw_rate,
                            RobotControlParameters* robot_params,
                            MIT_UserParameters* user_params,
                            GamepadCommand* gamepad_command) {
  const bool fsm_warmup = phase == ControllerPhase::kFsmWarmup;
  const bool balance_stand = phase == ControllerPhase::kBalanceStand;
  const bool controller_stand = phase == ControllerPhase::kControllerStand;
  if (fsm_warmup) {
    robot_params->control_mode = kStandUpControlMode;
  } else if (balance_stand || controller_stand) {
    robot_params->control_mode = kBalanceStandControlMode;
  } else {
    robot_params->control_mode = kLocomotionControlMode;
  }
  if (!fsm_warmup && !balance_stand && !controller_stand &&
      IsLegBotStationaryTrot(gait)) {
    vx = 0.;
    vy = 0.;
    yaw_rate = 0.;
  }
  robot_params->use_rc = 0;
  user_params->cmpc_gait = (fsm_warmup || balance_stand || controller_stand)
                                ? 4.0
                                : gait;
  gamepad_command->leftStickAnalog[0] =
      static_cast<float>((fsm_warmup || balance_stand || controller_stand)
                             ? 0.
                             : vy);
  gamepad_command->leftStickAnalog[1] =
      static_cast<float>((fsm_warmup || balance_stand || controller_stand)
                             ? 0.
                             : vx);
  gamepad_command->rightStickAnalog[0] =
      static_cast<float>((fsm_warmup || balance_stand || controller_stand)
                             ? 0.
                             : yaw_rate);
  gamepad_command->rightStickAnalog[1] = 0.f;
}

void ApplyControllerTestCommand(const legbot::BridgeArgs& args,
                                ControllerPhase phase,
                                RobotControlParameters* robot_params,
                                MIT_UserParameters* user_params,
                                GamepadCommand* gamepad_command,
                                double control_elapsed_s) {
  const bool fsm_warmup = phase == ControllerPhase::kFsmWarmup;
  const bool balance_stand = phase == ControllerPhase::kBalanceStand;
  const bool controller_stand = phase == ControllerPhase::kControllerStand;
  if (!args.test_locomotion && !balance_stand && !controller_stand &&
      !fsm_warmup) return;
  (void)control_elapsed_s;
  ApplyControllerCommand(phase, args.test_gait, args.test_vx,
                         args.test_vy, args.test_yaw_rate,
                         robot_params, user_params, gamepad_command);
}

bool HasControllerCommand(const legbot::DdsCommandArrays& command) {
  constexpr double kEps = 1.e-6;
  double max_abs_q = 0.;
  double max_abs_kp = 0.;
  double max_abs_kd = 0.;
  double max_abs_tau = 0.;
  for (std::size_t i = 0; i < legbot::kNumJoints; ++i) {
    if (!std::isfinite(command.q[i]) || !std::isfinite(command.dq[i]) ||
        !std::isfinite(command.kp[i]) || !std::isfinite(command.kd[i]) ||
        !std::isfinite(command.tau[i])) {
      return false;
    }
    max_abs_q = std::max(max_abs_q, std::abs(command.q[i]));
    max_abs_kp = std::max(max_abs_kp, std::abs(command.kp[i]));
    max_abs_kd = std::max(max_abs_kd, std::abs(command.kd[i]));
    max_abs_tau = std::max(max_abs_tau, std::abs(command.tau[i]));
  }
  return max_abs_q > kEps || max_abs_kp > kEps || max_abs_kd > kEps ||
         max_abs_tau > kEps;
}

struct MitOutputReadiness {
  bool ready = false;
  std::string reason;
  BridgeRejectCode reject_code = BridgeRejectCode::kOk;
  double max_abs_q = 0.;
  double max_kp = 0.;
  double max_kd = 0.;
};

const char* BridgeRejectCodeText(BridgeRejectCode code) {
  switch (code) {
    case BridgeRejectCode::kOk:
      return "ok";
    case BridgeRejectCode::kQDesNotFinite:
      return "q_des_not_finite";
    case BridgeRejectCode::kQDesOutOfRange:
      return "q_des_out_of_range";
    case BridgeRejectCode::kQDesJump:
      return "q_des_jump";
    case BridgeRejectCode::kQdDesTooLarge:
      return "qd_des_too_large";
    case BridgeRejectCode::kRearThighTooLow:
      return "rear_thigh_too_low";
    case BridgeRejectCode::kPreTrotTimeout:
      return "pre_trot_timeout";
    case BridgeRejectCode::kContactStateNotReady:
      return "contact_state_not_ready";
    case BridgeRejectCode::kSwingPhaseNotReady:
      return "swing_phase_not_ready";
    case BridgeRejectCode::kFootTargetJump:
      return "foot_target_jump";
    case BridgeRejectCode::kTiltTooLarge:
      return "tilt_too_large";
    case BridgeRejectCode::kWbcForceTooLow:
      return "wbc_force_too_low";
  }
  return "unknown";
}

const char* Key3PhaseText(Key3Phase phase) {
  switch (phase) {
    case Key3Phase::kNone:
      return "NONE";
    case Key3Phase::kMitForwardArming:
      return "MIT_FORWARD_ARMING";
    case Key3Phase::kLocomotionGait4Shadow:
      return "LOCOMOTION_GAIT4_SHADOW";
    case Key3Phase::kLocomotionGait4Direct:
      return "LOCOMOTION_GAIT4_DIRECT";
    case Key3Phase::kGait0Shadow:
      return "GAIT0_SHADOW";
    case Key3Phase::kGait0Blend:
      return "GAIT0_BLEND";
    case Key3Phase::kGait0Direct:
      return "GAIT0_DIRECT";
  }
  return "UNKNOWN";
}

double ReadyMaxQErrorForDdsJoint(int dds_joint,
                                 bool gait0_shadow_entry_check = false) {
  const int model = legbot::DdsToModelJoint(dds_joint);
  const int joint = model % static_cast<int>(legbot::kNumJointsPerLeg);
  if (joint == static_cast<int>(legbot::ModelJoint::Hip)) {
    if (gait0_shadow_entry_check) return kGait0ShadowMaxAbsQErrorHip;
    return kReadyMaxAbsQErrorHip;
  }
  if (joint == static_cast<int>(legbot::ModelJoint::Thigh)) {
    if (gait0_shadow_entry_check) return kGait0ShadowMaxAbsQErrorThigh;
    return kReadyMaxAbsQErrorThigh;
  }
  if (gait0_shadow_entry_check) return kGait0ShadowMaxAbsQErrorCalf;
  return kReadyMaxAbsQErrorCalf;
}

double Gait0RawVsHoldTakeoverLimitForDdsJoint(int dds_joint) {
  const int model = legbot::DdsToModelJoint(dds_joint);
  const int joint = model % static_cast<int>(legbot::kNumJointsPerLeg);
  if (joint == static_cast<int>(legbot::ModelJoint::Hip)) {
    return kGait0RawVsHoldTakeoverHip;
  }
  if (joint == static_cast<int>(legbot::ModelJoint::Thigh)) {
    return kGait0RawVsHoldTakeoverThigh;
  }
  return kGait0RawVsHoldTakeoverCalf;
}

double Gait0BlendAlphaScaleForDdsJoint(int dds_joint) {
  const int model = legbot::DdsToModelJoint(dds_joint);
  const int joint = model % static_cast<int>(legbot::kNumJointsPerLeg);
  if (joint == static_cast<int>(legbot::ModelJoint::Hip)) {
    return kGait0BlendHipAlphaScale;
  }
  if (joint == static_cast<int>(legbot::ModelJoint::Thigh)) {
    return kGait0BlendThighAlphaScale;
  }
  return kGait0BlendCalfAlphaScale;
}

double Gait0BlendMaxQStepForDdsJoint(int dds_joint) {
  const int model = legbot::DdsToModelJoint(dds_joint);
  const int joint = model % static_cast<int>(legbot::kNumJointsPerLeg);
  if (joint == static_cast<int>(legbot::ModelJoint::Hip)) {
    return kGait0BlendHipMaxQStep;
  }
  if (joint == static_cast<int>(legbot::ModelJoint::Thigh)) {
    return kGait0BlendThighMaxQStep;
  }
  return kGait0BlendCalfMaxQStep;
}

double Gait0BlendTauAlphaScaleForDdsJoint(int dds_joint) {
  const int model = legbot::DdsToModelJoint(dds_joint);
  const int joint = model % static_cast<int>(legbot::kNumJointsPerLeg);
  if (joint == static_cast<int>(legbot::ModelJoint::Hip)) {
    return kGait0BlendHipTauAlphaScale;
  }
  if (joint == static_cast<int>(legbot::ModelJoint::Thigh)) {
    return kGait0BlendThighTauAlphaScale;
  }
  return kGait0BlendCalfTauAlphaScale;
}

double Gait0BlendStanceKpMinForDdsJoint(int dds_joint) {
  const int model = legbot::DdsToModelJoint(dds_joint);
  const int joint = model % static_cast<int>(legbot::kNumJointsPerLeg);
  if (joint == static_cast<int>(legbot::ModelJoint::Hip)) {
    return kGait0BlendStanceHipKpMin;
  }
  if (joint == static_cast<int>(legbot::ModelJoint::Thigh)) {
    return kGait0BlendStanceThighKpMin;
  }
  return kGait0BlendStanceCalfKpMin;
}

double Gait0BlendStanceKdMinForDdsJoint(int dds_joint) {
  const int model = legbot::DdsToModelJoint(dds_joint);
  const int joint = model % static_cast<int>(legbot::kNumJointsPerLeg);
  if (joint == static_cast<int>(legbot::ModelJoint::Hip)) {
    return kGait0BlendStanceHipKdMin;
  }
  if (joint == static_cast<int>(legbot::ModelJoint::Thigh)) {
    return kGait0BlendStanceThighKdMin;
  }
  return kGait0BlendStanceCalfKdMin;
}

bool IsRearThighDdsJoint(int dds_joint) {
  return dds_joint == legbot::DdsJointIndexFromModel(
                          static_cast<int>(legbot::ModelLeg::RR),
                          static_cast<int>(legbot::ModelJoint::Thigh)) ||
         dds_joint == legbot::DdsJointIndexFromModel(
                          static_cast<int>(legbot::ModelLeg::RL),
                          static_cast<int>(legbot::ModelJoint::Thigh));
}

std::string RawRejectReason(BridgeRejectCode code, int joint, double q_fb,
                            double q_des, double value, double limit) {
  std::ostringstream oss;
  oss << BridgeRejectCodeText(code);
  if (joint >= 0) {
    oss << " joint=" << legbot::JointNameDds(joint) << " q_fb=" << std::fixed
        << std::setprecision(4) << q_fb << " q_des=" << q_des;
    if (code == BridgeRejectCode::kQDesJump) {
      oss << " diff=" << value << " limit=" << limit;
    } else if (code == BridgeRejectCode::kQdDesTooLarge) {
      oss << " qd_des=" << value << " limit=" << limit;
    } else if (code == BridgeRejectCode::kQDesOutOfRange) {
      oss << " limit=[" << value << "," << limit << "]";
    } else if (code == BridgeRejectCode::kRearThighTooLow) {
      oss << " min=" << limit;
    }
  }
  return oss.str();
}

MitOutputReadiness InspectLocomotionRawCommand(
    const legbot::DdsCommandArrays& command,
    const std::array<double, legbot::kNumJoints>& q_feedback,
    const legbot::BridgeArgs& args, double max_abs_qd_des,
    double rear_thigh_min_raw, bool gait0_shadow_entry_check) {
  MitOutputReadiness out;
  out.reason = "ok";
  for (std::size_t i = 0; i < legbot::kNumJoints; ++i) {
    const int dds = static_cast<int>(i);
    const int model = legbot::DdsToModelJoint(dds);
    if (!std::isfinite(command.q[i]) || !std::isfinite(command.dq[i]) ||
        !std::isfinite(command.kp[i]) || !std::isfinite(command.kd[i]) ||
        !std::isfinite(command.tau[i]) || !std::isfinite(q_feedback[i])) {
      out.reason = std::string("command_nonfinite:") + legbot::JointNameDds(dds);
      out.reject_code = BridgeRejectCode::kQDesNotFinite;
      return out;
    }
    out.max_abs_q = std::max(out.max_abs_q, std::abs(command.q[i]));
    out.max_kp = std::max(out.max_kp, command.kp[i]);
    out.max_kd = std::max(out.max_kd, command.kd[i]);

    const auto q_limit = legbot::CmdQDesLimitForModelJoint(model, args);
    if (command.q[i] < q_limit.first || command.q[i] > q_limit.second) {
      out.reason = RawRejectReason(BridgeRejectCode::kQDesOutOfRange, dds,
                                   q_feedback[i], command.q[i],
                                   q_limit.first, q_limit.second);
      out.reject_code = BridgeRejectCode::kQDesOutOfRange;
      return out;
    }

    if (std::abs(command.dq[i]) > max_abs_qd_des) {
      out.reason = RawRejectReason(BridgeRejectCode::kQdDesTooLarge, dds,
                                   q_feedback[i], command.q[i],
                                   std::abs(command.dq[i]), max_abs_qd_des);
      out.reject_code = BridgeRejectCode::kQdDesTooLarge;
      return out;
    }

    const double q_diff = std::abs(command.q[i] - q_feedback[i]);
    const double q_diff_limit =
        ReadyMaxQErrorForDdsJoint(dds, gait0_shadow_entry_check);
    if (q_diff > q_diff_limit) {
      out.reason = RawRejectReason(BridgeRejectCode::kQDesJump, dds,
                                   q_feedback[i], command.q[i], q_diff,
                                   q_diff_limit);
      out.reject_code = BridgeRejectCode::kQDesJump;
      return out;
    }

    if (IsRearThighDdsJoint(dds) && command.q[i] < rear_thigh_min_raw) {
      out.reason = RawRejectReason(BridgeRejectCode::kRearThighTooLow, dds,
                                   q_feedback[i], command.q[i], command.q[i],
                                   rear_thigh_min_raw);
      out.reject_code = BridgeRejectCode::kRearThighTooLow;
      return out;
    }
  }
  if (out.max_abs_q < 0.05) {
    out.reason = "q_des_all_near_zero";
    out.reject_code = BridgeRejectCode::kQDesNotFinite;
    return out;
  }
  out.ready = true;
  return out;
}

struct JointDiffDiagnostic {
  int joint = -1;
  double value = 0.;
  double q_feedback = 0.;
  double q_des = 0.;
};

std::array<double, 3> Vec3TextArray(const Vec3<float>& v);
std::array<double, 4> Vec4TextArray(const Vec4<float>& v);
std::array<double, 4> Vec4iTextArray(const Eigen::Vector4i& v);

struct Gait0MpcTrace {
  bool valid = false;
  std::array<double, 4> mpc_table_now{};
  std::array<double, 4> swing_phase{};
  std::array<double, 4> first_swing{};
};

Gait0MpcTrace CaptureGait0MpcTrace(const ConvexMPCDebugSnapshot* cmpc) {
  Gait0MpcTrace trace;
  if (!cmpc || !cmpc->valid) return trace;
  trace.valid = true;
  trace.mpc_table_now = Vec4iTextArray(cmpc->mpcTableNow);
  trace.swing_phase = Vec4TextArray(cmpc->swingStates);
  trace.first_swing = Vec4iTextArray(cmpc->firstSwing);
  return trace;
}

std::array<double, legbot::kNumJoints> AbsQStepByJoint(
    const legbot::DdsCommandArrays& command,
    const std::array<double, legbot::kNumJoints>& previous_q) {
  std::array<double, legbot::kNumJoints> out{};
  for (std::size_t i = 0; i < legbot::kNumJoints; ++i) {
    if (std::isfinite(command.q[i]) && std::isfinite(previous_q[i])) {
      out[i] = std::abs(command.q[i] - previous_q[i]);
    } else {
      out[i] = std::numeric_limits<double>::quiet_NaN();
    }
  }
  return out;
}

void PrintRawQStepTooLargeEvent(
    const JointDiffDiagnostic& raw_q_step,
    const legbot::DdsCommandArrays& raw_command,
    const std::array<double, legbot::kNumJoints>& previous_raw_q,
    const legbot::LowStateSnapshot& snapshot,
    const Gait0MpcTrace& prev_trace,
    const Gait0MpcTrace& curr_trace,
    const ConvexMPCDebugSnapshot* cmpc,
    const RobotRunner* runner) {
  const auto raw_q_step_by_joint =
      AbsQStepByJoint(raw_command, previous_raw_q);
  std::cout << "[LEGBOT-DDS][GAIT0-SHADOW] event=raw_q_step_too_large"
            << " joint="
            << (raw_q_step.joint >= 0 ? legbot::JointNameDds(raw_q_step.joint)
                                      : "unknown")
            << " joint_index=" << raw_q_step.joint
            << " step=" << raw_q_step.value
            << " limit=" << kGait0RawStableQStepLimit
            << " prev_raw_q=" << legbot::ArrayText(previous_raw_q, 5)
            << " curr_raw_q=" << legbot::ArrayText(raw_command.q, 5)
            << " raw_q_step_by_joint="
            << legbot::ArrayText(raw_q_step_by_joint, 5)
            << " q_feedback=" << legbot::ArrayText(snapshot.dds_q, 5)
            << "\n";
  std::cout << "  prev_mpc_table_now="
            << (prev_trace.valid ? legbot::ArrayText(prev_trace.mpc_table_now, 0)
                                 : "unavailable")
            << " curr_mpc_table_now="
            << (curr_trace.valid ? legbot::ArrayText(curr_trace.mpc_table_now, 0)
                                 : "unavailable")
            << " prev_swing_phase="
            << (prev_trace.valid ? legbot::ArrayText(prev_trace.swing_phase, 3)
                                 : "unavailable")
            << " curr_swing_phase="
            << (curr_trace.valid ? legbot::ArrayText(curr_trace.swing_phase, 3)
                                 : "unavailable")
            << " prev_firstSwing="
            << (prev_trace.valid ? legbot::ArrayText(prev_trace.first_swing, 0)
                                 : "unavailable")
            << " curr_firstSwing="
            << (curr_trace.valid ? legbot::ArrayText(curr_trace.first_swing, 0)
                                 : "unavailable")
            << "\n";
  if (cmpc && cmpc->valid) {
    for (int leg = 0; leg < 4; ++leg) {
      std::array<double, 3> diff{};
      for (int axis = 0; axis < 3; ++axis) {
        diff[axis] = static_cast<double>(cmpc->pFootDes[leg][axis] -
                                         cmpc->pFoot[leg][axis]);
      }
      std::cout << "  cmpc_foot model_leg=" << leg
                << " p=" << legbot::ArrayText(Vec3TextArray(cmpc->pFoot[leg]), 5)
                << " pDes="
                << legbot::ArrayText(Vec3TextArray(cmpc->pFootDes[leg]), 5)
                << " vDes="
                << legbot::ArrayText(Vec3TextArray(cmpc->vFootDes[leg]), 5)
                << " swing_p0="
                << legbot::ArrayText(Vec3TextArray(cmpc->swingP0[leg]), 5)
                << " swing_pf="
                << legbot::ArrayText(Vec3TextArray(cmpc->swingPf[leg]), 5)
                << " diff=" << legbot::ArrayText(diff, 5) << "\n";
    }
  } else {
    std::cout << "  cmpc_foot=unavailable\n";
  }
  const auto* leg_controller = runner ? runner->debugLegController() : nullptr;
  if (!leg_controller) {
    std::cout << "  leg_io=unavailable\n";
    return;
  }
  for (int leg = 0; leg < 4; ++leg) {
    const auto& data = leg_controller->datas[leg];
    const auto& cmd = leg_controller->commands[leg];
    std::cout << "  leg_io model_leg=" << leg
              << " p=" << legbot::ArrayText(Vec3TextArray(data.p), 5)
              << " pDes=" << legbot::ArrayText(Vec3TextArray(cmd.pDes), 5)
              << " vDes=" << legbot::ArrayText(Vec3TextArray(cmd.vDes), 5)
              << " qDes=" << legbot::ArrayText(Vec3TextArray(cmd.qDes), 5)
              << " tauFF="
              << legbot::ArrayText(Vec3TextArray(cmd.tauFeedForward), 5)
              << "\n";
  }
}

MitOutputReadiness InspectGait0ShadowReleaseGate(
    const legbot::DdsCommandArrays& raw_command,
    const std::array<double, legbot::kNumJoints>& entry_q_feedback,
    const legbot::LowStateSnapshot& snapshot,
    const legbot::LegBotDDSStateAdapter& state_adapter,
    const legbot::BridgeArgs& args,
    const JointDiffDiagnostic& raw_q_step,
    bool has_previous_raw_q,
    double max_abs_raw_qd,
    double max_abs_raw_tau,
    double foot_jump) {
  MitOutputReadiness out;
  out.reason = "ok";
  for (std::size_t i = 0; i < legbot::kNumJoints; ++i) {
    const int dds = static_cast<int>(i);
    const int model = legbot::DdsToModelJoint(dds);
    if (!std::isfinite(raw_command.q[i]) ||
        !std::isfinite(raw_command.dq[i]) ||
        !std::isfinite(raw_command.kp[i]) ||
        !std::isfinite(raw_command.kd[i]) ||
        !std::isfinite(raw_command.tau[i]) ||
        !std::isfinite(entry_q_feedback[i])) {
      out.reason = std::string("gait0_raw_nonfinite:") +
                   legbot::JointNameDds(dds);
      out.reject_code = BridgeRejectCode::kQDesNotFinite;
      return out;
    }
    out.max_abs_q = std::max(out.max_abs_q, std::abs(raw_command.q[i]));
    out.max_kp = std::max(out.max_kp, raw_command.kp[i]);
    out.max_kd = std::max(out.max_kd, raw_command.kd[i]);

    const auto q_limit = legbot::CmdQDesLimitForModelJoint(model, args);
    if (raw_command.q[i] < q_limit.first ||
        raw_command.q[i] > q_limit.second) {
      out.reason = RawRejectReason(BridgeRejectCode::kQDesOutOfRange, dds,
                                   entry_q_feedback[i], raw_command.q[i],
                                   q_limit.first, q_limit.second);
      out.reject_code = BridgeRejectCode::kQDesOutOfRange;
      return out;
    }

    const double entry_diff =
        std::abs(raw_command.q[i] - entry_q_feedback[i]);
    const double entry_limit = ReadyMaxQErrorForDdsJoint(
        dds, true /* gait0_shadow_entry_check */);
    if (entry_diff > entry_limit) {
      out.reason = RawRejectReason(BridgeRejectCode::kQDesJump, dds,
                                   entry_q_feedback[i], raw_command.q[i],
                                   entry_diff, entry_limit);
      out.reject_code = BridgeRejectCode::kQDesJump;
      return out;
    }
  }

  if (out.max_abs_q < 0.05) {
    out.reason = "q_des_all_near_zero";
    out.reject_code = BridgeRejectCode::kQDesNotFinite;
    return out;
  }

  if (!has_previous_raw_q || raw_q_step.joint < 0) {
    out.reason = "raw_q_step_unavailable";
    out.reject_code = BridgeRejectCode::kPreTrotTimeout;
    return out;
  }
  if (!std::isfinite(raw_q_step.value) ||
      raw_q_step.value >= kGait0RawStableQStepLimit) {
    std::ostringstream oss;
    oss << "raw_q_step_too_large joint="
        << (raw_q_step.joint >= 0 ? legbot::JointNameDds(raw_q_step.joint)
                                  : "unknown")
        << " step=" << raw_q_step.value
        << " limit=" << kGait0RawStableQStepLimit;
    out.reason = oss.str();
    out.reject_code = BridgeRejectCode::kQDesJump;
    return out;
  }
  if (!std::isfinite(max_abs_raw_qd) ||
      max_abs_raw_qd >= kGait0RawStableMaxAbsQd) {
    std::ostringstream oss;
    oss << "raw_qd_too_large max_abs_raw_qd=" << max_abs_raw_qd
        << " limit=" << kGait0RawStableMaxAbsQd;
    out.reason = oss.str();
    out.reject_code = BridgeRejectCode::kQdDesTooLarge;
    return out;
  }
  if (args.cmd_tau_limit > 0. &&
      (!std::isfinite(max_abs_raw_tau) ||
       max_abs_raw_tau >= args.cmd_tau_limit)) {
    std::ostringstream oss;
    oss << "raw_tau_too_large max_abs_raw_tau=" << max_abs_raw_tau
        << " limit=" << args.cmd_tau_limit;
    out.reason = oss.str();
    out.reject_code = BridgeRejectCode::kWbcForceTooLow;
    return out;
  }

  const double tilt = state_adapter.Tilt(snapshot);
  if (!std::isfinite(tilt) || tilt > kKey3TiltMaxRad) {
    std::ostringstream oss;
    oss << BridgeRejectCodeText(BridgeRejectCode::kTiltTooLarge)
        << " tilt=" << tilt << " limit=" << kKey3TiltMaxRad;
    out.reason = oss.str();
    out.reject_code = BridgeRejectCode::kTiltTooLarge;
    return out;
  }

  if (std::isfinite(foot_jump) && foot_jump > kFootTargetJumpHardLimitGait0) {
    std::ostringstream oss;
    oss << BridgeRejectCodeText(BridgeRejectCode::kFootTargetJump)
        << " max_abs_axis_jump=" << foot_jump
        << " foot_jump_hard_limit=" << kFootTargetJumpHardLimitGait0;
    out.reason = oss.str();
    out.reject_code = BridgeRejectCode::kFootTargetJump;
    return out;
  }

  out.ready = true;
  out.reject_code = BridgeRejectCode::kOk;
  return out;
}

MitOutputReadiness InspectMitOutputReadiness(
    const legbot::DdsCommandArrays& command) {
  MitOutputReadiness out;
  out.reason = "ok";
  for (std::size_t i = 0; i < legbot::kNumJoints; ++i) {
    if (!std::isfinite(command.q[i]) || !std::isfinite(command.dq[i]) ||
        !std::isfinite(command.kp[i]) || !std::isfinite(command.kd[i]) ||
        !std::isfinite(command.tau[i])) {
      out.reason = std::string("command_nonfinite:") +
                   legbot::JointNameDds(static_cast<int>(i));
      return out;
    }
    out.max_abs_q = std::max(out.max_abs_q, std::abs(command.q[i]));
    out.max_kp = std::max(out.max_kp, command.kp[i]);
    out.max_kd = std::max(out.max_kd, command.kd[i]);
  }
  if (out.max_kp < 1.0 || out.max_kd < 0.1) {
    out.reason = "low_kp_kd";
    return out;
  }
  if (out.max_abs_q < 0.05) {
    out.reason = "q_des_all_near_zero";
    return out;
  }
  out.ready = true;
  return out;
}

double DebugWbcSumFz(const RobotRunner* runner) {
  const auto* leg_controller = runner ? runner->debugLegController() : nullptr;
  if (!leg_controller) return std::numeric_limits<double>::quiet_NaN();
  double sum = 0.;
  for (int leg = 0; leg < 4; ++leg) {
    sum += static_cast<double>(
        leg_controller->commands[leg].forceFeedForward[2]);
  }
  return sum;
}

double DebugMaxFootPDesJump(const RobotRunner* runner) {
  const auto* leg_controller = runner ? runner->debugLegController() : nullptr;
  if (!leg_controller) return std::numeric_limits<double>::quiet_NaN();
  double max_jump = 0.;
  for (int leg = 0; leg < 4; ++leg) {
    const auto& data = leg_controller->datas[leg];
    const auto& cmd = leg_controller->commands[leg];
    for (int axis = 0; axis < 3; ++axis) {
      max_jump = std::max(
          max_jump,
          std::abs(static_cast<double>(cmd.pDes[axis] - data.p[axis])));
    }
  }
  return max_jump;
}

double DebugMaxCmpcFootPDesJump(const ConvexMPCDebugSnapshot* cmpc) {
  if (!cmpc || !cmpc->valid) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  double max_jump = 0.;
  for (int leg = 0; leg < 4; ++leg) {
    for (int axis = 0; axis < 3; ++axis) {
      max_jump = std::max(
          max_jump,
          std::abs(static_cast<double>(cmpc->pFootDes[leg][axis] -
                                       cmpc->pFoot[leg][axis])));
    }
  }
  return max_jump;
}

double DebugCmpcSumFrDesZ(const ConvexMPCDebugSnapshot* cmpc) {
  if (!cmpc || !cmpc->valid) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  double sum = 0.;
  for (int leg = 0; leg < 4; ++leg) {
    sum += static_cast<double>(cmpc->FrDes[leg][2]);
  }
  return sum;
}

std::array<double, 3> Vec3TextArray(const Vec3<float>& v);

JointDiffDiagnostic MaxAbsQDesFeedbackDiff(
    const legbot::DdsCommandArrays& raw_command,
    const legbot::LowStateSnapshot& snapshot) {
  JointDiffDiagnostic out;
  for (std::size_t i = 0; i < legbot::kNumJoints; ++i) {
    if (!std::isfinite(raw_command.q[i]) ||
        !std::isfinite(snapshot.dds_q[i])) {
      continue;
    }
    const double diff = std::abs(raw_command.q[i] - snapshot.dds_q[i]);
    if (out.joint < 0 || diff > out.value) {
      out.joint = static_cast<int>(i);
      out.value = diff;
      out.q_feedback = snapshot.dds_q[i];
      out.q_des = raw_command.q[i];
    }
  }
  return out;
}

JointDiffDiagnostic MaxAbsQStep(
    const legbot::DdsCommandArrays& command,
    const std::array<double, legbot::kNumJoints>& previous_q) {
  JointDiffDiagnostic out;
  for (std::size_t i = 0; i < legbot::kNumJoints; ++i) {
    if (!std::isfinite(command.q[i]) || !std::isfinite(previous_q[i])) {
      continue;
    }
    const double step = std::abs(command.q[i] - previous_q[i]);
    if (out.joint < 0 || step > out.value) {
      out.joint = static_cast<int>(i);
      out.value = step;
      out.q_feedback = previous_q[i];
      out.q_des = command.q[i];
    }
  }
  return out;
}

double MaxAbsRawQd(const legbot::DdsCommandArrays& command) {
  double max_abs = 0.;
  for (double qd : command.dq) {
    if (!std::isfinite(qd)) return std::numeric_limits<double>::quiet_NaN();
    max_abs = std::max(max_abs, std::abs(qd));
  }
  return max_abs;
}

double MaxAbsPublishedQd(const legbot::DdsCommandArrays& command) {
  double max_abs = 0.;
  for (double qd : command.dq) {
    if (!std::isfinite(qd)) return std::numeric_limits<double>::quiet_NaN();
    max_abs = std::max(max_abs, std::abs(qd));
  }
  return max_abs;
}

double MaxAbsRawTau(const legbot::DdsCommandArrays& command) {
  double max_abs = 0.;
  for (double tau : command.tau) {
    if (!std::isfinite(tau)) return std::numeric_limits<double>::quiet_NaN();
    max_abs = std::max(max_abs, std::abs(tau));
  }
  return max_abs;
}

double MaxAbsPublishedQFeedbackDiff(
    const legbot::DdsCommandArrays& command,
    const legbot::LowStateSnapshot& snapshot) {
  double max_diff = 0.;
  for (std::size_t i = 0; i < legbot::kNumJoints; ++i) {
    if (!std::isfinite(command.q[i]) ||
        !std::isfinite(snapshot.dds_q[i])) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    max_diff = std::max(max_diff, std::abs(command.q[i] - snapshot.dds_q[i]));
  }
  return max_diff;
}

double MaxAbsDqFeedback(const legbot::LowStateSnapshot& snapshot) {
  double max_dq = 0.;
  for (double dq : snapshot.dds_dq) {
    if (!std::isfinite(dq)) return std::numeric_limits<double>::quiet_NaN();
    max_dq = std::max(max_dq, std::abs(dq));
  }
  return max_dq;
}

void PrintGait0EntryDiagnostics(
    const legbot::DdsCommandArrays& raw_command,
    const legbot::LowStateSnapshot& snapshot,
    const RobotRunner* runner, const MIT_Controller* controller) {
  std::cout << "[LEGBOT-DDS][KEY3-PHASE] gait0_shadow_entry_diagnostics\n";
  const auto* cmpc = controller ? controller->debugConvexMPC() : nullptr;
  if (cmpc && cmpc->valid) {
    std::cout << "  cmpc_mpc_table_now="
              << legbot::ArrayText(std::array<double, 4>{
                     static_cast<double>(cmpc->mpcTableNow[0]),
                     static_cast<double>(cmpc->mpcTableNow[1]),
                     static_cast<double>(cmpc->mpcTableNow[2]),
                     static_cast<double>(cmpc->mpcTableNow[3])}, 0)
              << " swing_phase="
              << legbot::ArrayText(std::array<double, 4>{
                     static_cast<double>(cmpc->swingStates[0]),
                     static_cast<double>(cmpc->swingStates[1]),
                     static_cast<double>(cmpc->swingStates[2]),
                     static_cast<double>(cmpc->swingStates[3])}, 3)
              << " swing_time="
              << legbot::ArrayText(std::array<double, 4>{
                     static_cast<double>(cmpc->swingTimes[0]),
                     static_cast<double>(cmpc->swingTimes[1]),
                     static_cast<double>(cmpc->swingTimes[2]),
                     static_cast<double>(cmpc->swingTimes[3])}, 3)
              << " swing_time_remaining="
              << legbot::ArrayText(std::array<double, 4>{
                     static_cast<double>(cmpc->swingTimeRemaining[0]),
                     static_cast<double>(cmpc->swingTimeRemaining[1]),
                     static_cast<double>(cmpc->swingTimeRemaining[2]),
                     static_cast<double>(cmpc->swingTimeRemaining[3])}, 3)
              << " firstSwing="
              << legbot::ArrayText(std::array<double, 4>{
                     static_cast<double>(cmpc->firstSwing[0]),
                     static_cast<double>(cmpc->firstSwing[1]),
                     static_cast<double>(cmpc->firstSwing[2]),
                     static_cast<double>(cmpc->firstSwing[3])}, 0)
              << "\n";
    for (int leg = 0; leg < 4; ++leg) {
      std::array<double, 3> diff{};
      double max_axis_diff = 0.;
      for (int axis = 0; axis < 3; ++axis) {
        diff[axis] =
            static_cast<double>(cmpc->pFootDes[leg][axis] -
                                cmpc->pFoot[leg][axis]);
        max_axis_diff = std::max(max_axis_diff, std::abs(diff[axis]));
      }
      std::cout << "  cmpc_foot model_leg=" << leg
                << " p=" << legbot::ArrayText(Vec3TextArray(cmpc->pFoot[leg]), 5)
                << " pDes="
                << legbot::ArrayText(Vec3TextArray(cmpc->pFootDes[leg]), 5)
                << " diff=" << legbot::ArrayText(diff, 5)
                << " max_abs_axis_diff=" << max_axis_diff
                << " FrDes="
                << legbot::ArrayText(Vec3TextArray(cmpc->FrDes[leg]), 5)
                << "\n";
    }
  } else {
    std::cout << "  cmpc_debug=unavailable\n";
  }

  const auto qdiff = MaxAbsQDesFeedbackDiff(raw_command, snapshot);
  if (qdiff.joint >= 0) {
    std::cout << "  max_q_des_raw_feedback_diff joint="
              << legbot::JointNameDds(qdiff.joint)
              << " diff=" << std::fixed << std::setprecision(5)
              << qdiff.value << " q_feedback=" << qdiff.q_feedback
              << " q_des_raw=" << qdiff.q_des << "\n";
  } else {
    std::cout << "  max_q_des_raw_feedback_diff=unavailable\n";
  }

  const auto* leg_controller = runner ? runner->debugLegController() : nullptr;
  if (!leg_controller) {
    std::cout << "  leg_controller=unavailable\n";
    return;
  }
  for (int leg = 0; leg < 4; ++leg) {
    const auto& data = leg_controller->datas[leg];
    const auto& cmd = leg_controller->commands[leg];
    std::array<double, 3> diff{};
    double max_axis_diff = 0.;
    for (int axis = 0; axis < 3; ++axis) {
      diff[axis] =
          static_cast<double>(cmd.pDes[axis] - data.p[axis]);
      max_axis_diff = std::max(max_axis_diff, std::abs(diff[axis]));
    }
    std::cout << "  foot model_leg=" << leg
              << " p=" << legbot::ArrayText(std::array<double, 3>{
                     static_cast<double>(data.p[0]),
                     static_cast<double>(data.p[1]),
                     static_cast<double>(data.p[2])}, 5)
              << " pDes=" << legbot::ArrayText(std::array<double, 3>{
                     static_cast<double>(cmd.pDes[0]),
                     static_cast<double>(cmd.pDes[1]),
                     static_cast<double>(cmd.pDes[2])}, 5)
              << " diff=" << legbot::ArrayText(diff, 5)
              << " max_abs_axis_diff=" << max_axis_diff << "\n";
  }
}

MitOutputReadiness InspectKey3LocomotionReadiness(
    const legbot::DdsCommandArrays& raw_command,
    const legbot::LowStateSnapshot& snapshot,
    const legbot::LegBotDDSStateAdapter& state_adapter,
    const legbot::BridgeArgs& args, const RobotRunner* runner,
    const MIT_Controller* controller, bool gait4_allstance_check,
    bool gait0_shadow_entry_check,
    bool wbc_force_hard_gate,
    double raw_warmup_seconds, const char* raw_warmup_reason,
    double phase_elapsed) {
  if (raw_warmup_seconds > 0. &&
      phase_elapsed < raw_warmup_seconds) {
    MitOutputReadiness out;
    out.ready = false;
    out.reason = raw_warmup_reason ? raw_warmup_reason : "raw_warmup";
    out.reject_code = BridgeRejectCode::kPreTrotTimeout;
    return out;
  }

  MitOutputReadiness out = InspectLocomotionRawCommand(
      raw_command, snapshot.dds_q, args,
      gait4_allstance_check ? kReadyMaxAbsQdDes : kGaitEntryMaxAbsQdDes,
      gait4_allstance_check ? kPreTrotRearThighMinRaw
                            : kGaitEntryRearThighMinRaw,
      gait0_shadow_entry_check);
  if (!out.ready) return out;

  const double tilt = state_adapter.Tilt(snapshot);
  if (!std::isfinite(tilt) || tilt > kKey3TiltMaxRad) {
    out.ready = false;
    out.reject_code = BridgeRejectCode::kTiltTooLarge;
    std::ostringstream oss;
    oss << BridgeRejectCodeText(out.reject_code) << " tilt=" << tilt
        << " limit=" << kKey3TiltMaxRad;
    out.reason = oss.str();
    return out;
  }

  const auto* cmpc = controller ? controller->debugConvexMPC() : nullptr;
  if (!cmpc || !cmpc->valid) {
    out.ready = false;
    out.reject_code = BridgeRejectCode::kContactStateNotReady;
    out.reason = "cmpc_debug_unavailable";
    return out;
  }

  if (gait4_allstance_check) {
    for (int leg = 0; leg < 4; ++leg) {
      if (cmpc->mpcTableNow[leg] != 1) {
        out.ready = false;
        out.reject_code = BridgeRejectCode::kContactStateNotReady;
        std::ostringstream oss;
        oss << "gait4_mpc_table_not_allstance leg=" << leg
            << " contact=" << cmpc->contactStates[leg]
            << " mpc=" << cmpc->mpcTableNow[leg];
        out.reason = oss.str();
        return out;
      }
      if (std::abs(static_cast<double>(cmpc->swingStates[leg])) > 0.05) {
        out.ready = false;
        out.reject_code = BridgeRejectCode::kSwingPhaseNotReady;
        std::ostringstream oss;
        oss << "gait4_swing_phase_not_zero leg=" << leg
            << " phase=" << cmpc->swingStates[leg]
            << " contact=" << cmpc->contactStates[leg]
            << " mpc=" << cmpc->mpcTableNow[leg];
        out.reason = oss.str();
        return out;
      }
      if (std::abs(static_cast<double>(cmpc->swingTimes[leg])) > 1.e-4 ||
          std::abs(static_cast<double>(cmpc->swingTimeRemaining[leg])) >
              1.e-4) {
        out.ready = false;
        out.reject_code = BridgeRejectCode::kSwingPhaseNotReady;
        std::ostringstream oss;
        oss << "gait4_swing_time_not_zero leg=" << leg
            << " swing_time=" << cmpc->swingTimes[leg]
            << " swing_time_remaining=" << cmpc->swingTimeRemaining[leg]
            << " contact=" << cmpc->contactStates[leg]
            << " mpc=" << cmpc->mpcTableNow[leg];
        out.reason = oss.str();
        return out;
      }
    }
  } else {
    if (gait0_shadow_entry_check && phase_elapsed < 0.25) {
      for (int leg = 0; leg < 4; ++leg) {
        if (cmpc->swingStates[leg] > 0.75f) {
          out.ready = false;
          out.reject_code = BridgeRejectCode::kSwingPhaseNotReady;
          std::ostringstream oss;
          oss << "gait0_initial_swing_phase_late leg=" << leg
              << " phase=" << cmpc->swingStates[leg];
          out.reason = oss.str();
          return out;
        }
      }
    }
    for (int leg = 0; leg < 4; ++leg) {
      if (cmpc->swingTimes[leg] > 0.f &&
          std::abs(static_cast<double>(cmpc->swingTimes[leg]) - 0.26) > 0.08) {
        out.ready = false;
        out.reject_code = BridgeRejectCode::kSwingPhaseNotReady;
        std::ostringstream oss;
        oss << "gait0_swing_time_unexpected leg=" << leg
            << " swing_time=" << cmpc->swingTimes[leg];
        out.reason = oss.str();
        return out;
      }
    }
  }

  const double leg_command_foot_jump = DebugMaxFootPDesJump(runner);
  const double cmpc_foot_jump = DebugMaxCmpcFootPDesJump(cmpc);
  const double foot_jump = (!gait4_allstance_check &&
                            std::isfinite(cmpc_foot_jump))
                               ? cmpc_foot_jump
                               : leg_command_foot_jump;
  const bool foot_jump_hard_gate = !gait4_allstance_check;
  const double foot_target_jump_hard_limit =
      foot_jump_hard_gate ? kFootTargetJumpHardLimitGait0
                          : kFootTargetJumpHardLimitDefault;
  if (foot_jump_hard_gate) {
    if (std::isfinite(foot_jump) &&
        foot_jump > foot_target_jump_hard_limit) {
      out.ready = false;
      out.reject_code = BridgeRejectCode::kFootTargetJump;
      std::ostringstream oss;
      oss << BridgeRejectCodeText(out.reject_code) << " max_abs_axis_jump="
          << foot_jump
          << " foot_jump_warn_limit=" << kFootTargetJumpWarnLimit
          << " foot_jump_hard_limit=" << foot_target_jump_hard_limit
          << " source="
          << (std::isfinite(cmpc_foot_jump) ? "cmpc_debug" : "leg_command")
          << " leg_command_jump=" << leg_command_foot_jump;
      out.reason = oss.str();
      return out;
    }
  }

  const double leg_command_sum_fz = DebugWbcSumFz(runner);
  const double cmpc_sum_fz = DebugCmpcSumFrDesZ(cmpc);
  const double sum_fz = (!gait4_allstance_check && std::isfinite(cmpc_sum_fz))
                            ? cmpc_sum_fz
                            : leg_command_sum_fz;
  const double wbc_force_hard_min =
      wbc_force_hard_gate ? kWbcForceHardMinFzGait0Direct
                          : kWbcForceWarnMinFz;
  if (!gait4_allstance_check && wbc_force_hard_gate &&
      std::isfinite(sum_fz) && sum_fz < wbc_force_hard_min) {
    out.ready = false;
    out.reject_code = BridgeRejectCode::kWbcForceTooLow;
    std::ostringstream oss;
    oss << BridgeRejectCodeText(out.reject_code) << " sum_fz=" << sum_fz
        << " wbc_force_warn_min=" << kWbcForceWarnMinFz
        << " wbc_force_hard_min=" << wbc_force_hard_min
        << " wbc_force_hard_gate=true"
        << " source="
        << (std::isfinite(cmpc_sum_fz) ? "cmpc_debug" : "leg_command")
        << " leg_command_sum_fz=" << leg_command_sum_fz;
    out.reason = oss.str();
    return out;
  }

  out.ready = true;
  out.reason = "ok";
  out.reject_code = BridgeRejectCode::kOk;
  return out;
}

bool PopulateRawCommandStats(const legbot::DdsCommandArrays& raw,
                             legbot::OutputGuardStats* guard) {
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
  for (std::size_t i = 0; i < legbot::kNumJoints; ++i) {
    if (!std::isfinite(raw.q[i]) || !std::isfinite(raw.dq[i]) ||
        !std::isfinite(raw.tau[i]) || !std::isfinite(raw.kp[i]) ||
        !std::isfinite(raw.kd[i])) {
      guard->fault_reason =
          std::string("raw_command_nonfinite:") +
          legbot::JointNameDds(static_cast<int>(i));
      return false;
    }
    guard->max_abs_tau_raw =
        std::max(guard->max_abs_tau_raw, std::abs(raw.tau[i]));
    guard->max_abs_tau_pub =
        std::max(guard->max_abs_tau_pub, std::abs(raw.tau[i]));
  }
  return true;
}

void PopulateRejectedHoldStats(const legbot::DdsCommandArrays& raw,
                               const legbot::DdsCommandArrays& hold,
                               legbot::OutputGuardStats* guard) {
  if (!guard) return;
  guard->raw = raw;
  guard->pub = hold;
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
  for (std::size_t i = 0; i < legbot::kNumJoints; ++i) {
    guard->max_abs_tau_raw =
        std::max(guard->max_abs_tau_raw, std::abs(raw.tau[i]));
    guard->max_abs_tau_pub =
        std::max(guard->max_abs_tau_pub, std::abs(hold.tau[i]));
  }
}

const char* FsmStateNameText(FSM_StateName state) {
  switch (state) {
    case FSM_StateName::INVALID:
      return "INVALID";
    case FSM_StateName::PASSIVE:
      return "PASSIVE";
    case FSM_StateName::JOINT_PD:
      return "JOINT_PD";
    case FSM_StateName::IMPEDANCE_CONTROL:
      return "IMPEDANCE_CONTROL";
    case FSM_StateName::STAND_UP:
      return "STAND_UP";
    case FSM_StateName::BALANCE_STAND:
      return "BALANCE_STAND";
    case FSM_StateName::LOCOMOTION:
      return "LOCOMOTION";
    case FSM_StateName::RECOVERY_STAND:
      return "RECOVERY_STAND";
    case FSM_StateName::VISION:
      return "VISION";
    case FSM_StateName::BACKFLIP:
      return "BACKFLIP";
    case FSM_StateName::FRONTJUMP:
      return "FRONTJUMP";
  }
  return "UNKNOWN";
}

FSM_StateName CurrentFsmStateName(const MIT_Controller* controller) {
  const auto* fsm = controller ? controller->debugControlFSM() : nullptr;
  if (!fsm || !fsm->currentState) return FSM_StateName::INVALID;
  return fsm->currentState->stateName;
}

std::array<double, 3> Vec3TextArray(const Vec3<float>& v) {
  return {static_cast<double>(v[0]), static_cast<double>(v[1]),
          static_cast<double>(v[2])};
}

std::array<double, 4> Vec4TextArray(const Vec4<float>& v) {
  return {static_cast<double>(v[0]), static_cast<double>(v[1]),
          static_cast<double>(v[2]), static_cast<double>(v[3])};
}

std::array<double, 4> Vec4iTextArray(const Eigen::Vector4i& v) {
  return {static_cast<double>(v[0]), static_cast<double>(v[1]),
          static_cast<double>(v[2]), static_cast<double>(v[3])};
}

std::array<double, 3> Mat3DiagTextArray(const Mat3<float>& m) {
  return {static_cast<double>(m(0, 0)), static_cast<double>(m(1, 1)),
          static_cast<double>(m(2, 2))};
}

const char* ModelLegName(int leg) {
  switch (leg) {
    case 0:
      return "FR";
    case 1:
      return "FL";
    case 2:
      return "RR";
    case 3:
      return "RL";
  }
  return "unknown";
}

const char* ModelJointName(int joint) {
  switch (joint) {
    case 0:
      return "Hip";
    case 1:
      return "Thigh";
    case 2:
      return "Calf";
  }
  return "unknown";
}

std::array<double, 3> Vec3fTextArray(const Vec3<float>& v) {
  return {static_cast<double>(v[0]), static_cast<double>(v[1]),
          static_cast<double>(v[2])};
}

Vec3<float> ComputeFootPosition(Quadruped<float>& quadruped,
                                const std::array<double, legbot::kNumJoints>& model_q,
                                int leg) {
  Vec3<float> q;
  q << static_cast<float>(
           model_q[legbot::ModelJointIndex(leg, static_cast<int>(
                                                   legbot::ModelJoint::Hip))]),
      static_cast<float>(
          model_q[legbot::ModelJointIndex(leg, static_cast<int>(
                                                  legbot::ModelJoint::Thigh))]),
      static_cast<float>(
          model_q[legbot::ModelJointIndex(leg, static_cast<int>(
                                                  legbot::ModelJoint::Calf))]);
  Vec3<float> p;
  computeLegJacobianAndPosition<float>(quadruped, q, nullptr, &p, leg);
  return p;
}

void PrintLegBotMappingAndFkDiagnostics(
    const legbot::LowStateSnapshot& snapshot,
    const LegController<float>* leg_controller,
    const char* tag) {
  std::cout << "[LEGBOT-DDS][MAPPING-DIAG] tag=" << (tag ? tag : "unknown")
            << "\n";
  const auto dds_q_back = legbot::ModelQToDdsQ(snapshot.model_q);
  double max_roundtrip_error = 0.;
  for (std::size_t i = 0; i < legbot::kNumJoints; ++i) {
    max_roundtrip_error = std::max(
        max_roundtrip_error, std::abs(dds_q_back[i] - snapshot.dds_q[i]));
  }
  std::cout << "  DDS index | DDS joint name | model_leg | model_joint | physical leg guess | q_dds | q_model\n";
  for (std::size_t dds = 0; dds < legbot::kNumJoints; ++dds) {
    const int model = legbot::DdsToModelJoint(static_cast<int>(dds));
    const int leg = model / static_cast<int>(legbot::kNumJointsPerLeg);
    const int joint = model % static_cast<int>(legbot::kNumJointsPerLeg);
    std::cout << "  " << dds << " " << legbot::JointNameDds(static_cast<int>(dds))
              << " model_leg=" << leg << "(" << ModelLegName(leg) << ")"
              << " model_joint=" << ModelJointName(joint)
              << " physical_leg_guess=" << ModelLegName(leg)
              << " q_dds=" << snapshot.dds_q[dds]
              << " q_model=" << snapshot.model_q[model] << "\n";
  }
  std::cout << "  model_leg 0 -> physical_guess=" << ModelLegName(0) << "\n"
            << "  model_leg 1 -> physical_guess=" << ModelLegName(1) << "\n"
            << "  model_leg 2 -> physical_guess=" << ModelLegName(2) << "\n"
            << "  model_leg 3 -> physical_guess=" << ModelLegName(3) << "\n";
  for (int leg = 0; leg < static_cast<int>(legbot::kNumLegs); ++leg) {
    std::cout << "  model_leg=" << leg << "(" << ModelLegName(leg) << ")"
              << " Hip DDS index="
              << legbot::DdsJointIndexFromModel(
                     leg, static_cast<int>(legbot::ModelJoint::Hip))
              << " Thigh DDS index="
              << legbot::DdsJointIndexFromModel(
                     leg, static_cast<int>(legbot::ModelJoint::Thigh))
              << " Calf DDS index="
              << legbot::DdsJointIndexFromModel(
                     leg, static_cast<int>(legbot::ModelJoint::Calf))
              << "\n";
  }
  std::cout << "  max_roundtrip_error=" << max_roundtrip_error
            << " dds_q=" << legbot::ArrayText(snapshot.dds_q, 5)
            << " model_q=" << legbot::ArrayText(snapshot.model_q, 5)
            << " dds_q_back=" << legbot::ArrayText(dds_q_back, 5) << "\n";

  if (!leg_controller) {
    std::cout << "  fk_diag=unavailable leg_controller=null\n";
    return;
  }
  auto& quadruped =
      const_cast<Quadruped<float>&>(leg_controller->_quadruped);
  std::cout << "  q_feedback=" << legbot::ArrayText(snapshot.dds_q, 5)
            << " current_q_model=" << legbot::ArrayText(snapshot.model_q, 5)
            << " body_height_estimate=" << kLegBotNominalHeight
            << " kLegBotNominalHeight=" << kLegBotNominalHeight
            << " abadLinkLength=" << quadruped._abadLinkLength
            << " hipLinkLength=" << quadruped._hipLinkLength
            << " kneeLinkLength=" << quadruped._kneeLinkLength
            << " kneeLinkY_offset=" << quadruped._kneeLinkY_offset
            << "\n";
  for (int leg = 0; leg < static_cast<int>(legbot::kNumLegs); ++leg) {
    const auto hip_location = quadruped.getHipLocation(leg);
    const auto foot = ComputeFootPosition(quadruped, snapshot.model_q, leg);
    std::cout << "  foot_p model_leg=" << leg << "(" << ModelLegName(leg)
              << ") leg_frame=" << legbot::ArrayText(Vec3fTextArray(foot), 5)
              << " hip_location="
              << legbot::ArrayText(Vec3fTextArray(hip_location), 5)
              << " sideSign=" << Quadruped<float>::getSideSign(leg)
              << "\n";
    for (int joint = 0; joint < static_cast<int>(legbot::kNumJointsPerLeg);
         ++joint) {
      auto perturbed_q = snapshot.model_q;
      perturbed_q[legbot::ModelJointIndex(leg, joint)] += 0.05;
      const auto perturbed_foot =
          ComputeFootPosition(quadruped, perturbed_q, leg);
      const auto delta = perturbed_foot - foot;
      std::cout << "  model_leg=" << leg << "(" << ModelLegName(leg) << ") "
                << ModelJointName(joint)
                << "+0.05 delta_foot="
                << legbot::ArrayText(Vec3fTextArray(delta), 5) << "\n";
    }
  }
}

void PrintConvexMPCDebug(const MIT_Controller* controller) {
  const auto* debug = controller ? controller->debugConvexMPC() : nullptr;
  if (!debug || !debug->valid) {
    std::cout << "cmpc_debug=unavailable\n";
    return;
  }
  std::cout << "cmpc_debug cmpc_gait_number=" << debug->gaitNumber
            << " cmpc_current_gait=" << debug->currentGait
            << " cmpc_gait_phase=" << debug->gaitPhase
            << " cmpc_iteration=" << debug->iterationCounter << "\n"
            << "  cmpc_desired_contact="
            << legbot::ArrayText(Vec4TextArray(debug->contactStates), 3)
            << " contactStates="
            << legbot::ArrayText(Vec4TextArray(debug->contactStates), 3)
            << " cmpc_swing_phase="
            << legbot::ArrayText(Vec4TextArray(debug->swingStates), 3)
            << " cmpc_se_contact_phase="
            << legbot::ArrayText(Vec4TextArray(debug->seContactState), 3)
            << "\n"
            << "  cmpc_swing_time="
            << legbot::ArrayText(Vec4TextArray(debug->swingTimes), 3)
            << " cmpc_swing_time_remaining="
            << legbot::ArrayText(Vec4TextArray(debug->swingTimeRemaining), 3)
            << "\n"
            << "  cmpc_mpc_table_now="
            << legbot::ArrayText(Vec4iTextArray(debug->mpcTableNow), 0)
            << " cmpc_first_swing="
            << legbot::ArrayText(Vec4iTextArray(debug->firstSwing), 0)
            << " firstSwing="
            << legbot::ArrayText(Vec4iTextArray(debug->firstSwing), 0)
            << "\n";
  for (int leg = 0; leg < 4; ++leg) {
    std::cout << "  cmpc_foot model_leg=" << leg
              << " p=" << legbot::ArrayText(Vec3TextArray(debug->pFoot[leg]), 5)
              << " pDes="
              << legbot::ArrayText(Vec3TextArray(debug->pFootDes[leg]), 5)
              << " vDes="
              << legbot::ArrayText(Vec3TextArray(debug->vFootDes[leg]), 5)
              << " FrDes="
              << legbot::ArrayText(Vec3TextArray(debug->FrDes[leg]), 5)
              << "\n";
  }
}

class TerminalRawMode {
 public:
  TerminalRawMode() {
    enabled_ = isatty(STDIN_FILENO) && tcgetattr(STDIN_FILENO, &old_) == 0;
    if (!enabled_) return;
    termios raw = old_;
    raw.c_lflag &= static_cast<unsigned>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) enabled_ = false;
  }
  ~TerminalRawMode() {
    if (enabled_) tcsetattr(STDIN_FILENO, TCSANOW, &old_);
  }
  TerminalRawMode(const TerminalRawMode&) = delete;
  TerminalRawMode& operator=(const TerminalRawMode&) = delete;

 private:
  bool enabled_ = false;
  termios old_{};
};

class CsvLogger {
 public:
  CsvLogger(const std::string& path, double hz,
            const legbot::Clock::time_point& t0)
      : enabled_(!path.empty()),
        period_s_(1. / std::max(1., hz)),
        t0_(t0),
        next_log_t_(t0) {
    if (!enabled_) return;
    out_.open(path);
    if (!out_) throw std::runtime_error("cannot open csv log: " + path);
    out_ << std::fixed << std::setprecision(6);
    WriteHeader();
    std::cout << "[LEGBOT-DDS] CSV log enabled: " << path
              << " hz=" << hz << "\n";
  }

  bool enabled() const { return enabled_; }

  void MaybeWrite(legbot::Clock::time_point now, const char* mode,
                  double requested_gait,
                  const MIT_UserParameters* user_params,
                  const RobotControlParameters* robot_params,
                  const legbot::LowStateSnapshot& snapshot,
                  const legbot::LegBotDDSStateAdapter& state_adapter,
                  const legbot::OutputGuardStats& guard,
                  const RobotRunner* runner,
                  const MIT_Controller* controller, bool bridge_ready = true,
                  BridgeRejectCode bridge_reject_code = BridgeRejectCode::kOk,
                  const char* bridge_stage = "DIRECT", double alpha_q = 1.0,
                  double alpha_qd = 1.0, double alpha_tau = 1.0) {
    if (!enabled_ || now < next_log_t_) return;
    next_log_t_ = now + std::chrono::duration_cast<legbot::Clock::duration>(
                            std::chrono::duration<double>(period_s_));

    const auto rpy = legbot::QuatWxyzToRpy(snapshot.imu_quat_wxyz);
    double max_abs_dq = 0.;
    double max_qerr = 0.;
    for (std::size_t i = 0; i < legbot::kNumJoints; ++i) {
      max_abs_dq = std::max(max_abs_dq, std::abs(snapshot.dds_dq[i]));
      max_qerr = std::max(max_qerr,
                          std::abs(guard.pub.q[i] - snapshot.dds_q[i]));
    }

    out_ << legbot::SecondsSince(t0_, now) << ',' << mode << ','
         << (bridge_ready ? 1 : 0) << ','
         << static_cast<int>(bridge_reject_code) << ',' << bridge_stage << ','
         << alpha_q << ',' << alpha_qd << ',' << alpha_tau << ','
         << requested_gait << ','
         << (user_params ? user_params->cmpc_gait : -1.0) << ','
         << (robot_params ? robot_params->control_mode : -1.0) << ','
         << state_adapter.Tilt(snapshot) << ',' << rpy[0] << ',' << rpy[1]
         << ',' << rpy[2] << ',' << snapshot.gyro[0] << ','
         << snapshot.gyro[1] << ',' << snapshot.gyro[2] << ','
         << guard.max_abs_tau_raw << ',' << guard.max_abs_tau_pub << ','
         << max_abs_dq << ',' << max_qerr;
    WriteArray(snapshot.dds_q);
    WriteArray(snapshot.dds_dq);
    WriteArray(guard.raw.q);
    WriteArray(guard.pub.q);
    WriteArray(guard.raw.dq);
    WriteArray(guard.pub.dq);
    WriteArray(guard.raw.tau);
    WriteArray(guard.pub.tau);
    WriteArray(guard.pub.kp);
    WriteArray(guard.pub.kd);

    if (runner) {
      const auto& se = runner->debugStateEstimate();
      WriteVec3(se.rpy);
      WriteVec3(se.position);
      WriteVec3(se.vWorld);
      out_ << ',' << se.contactEstimate[0] << ',' << se.contactEstimate[1]
           << ',' << se.contactEstimate[2] << ',' << se.contactEstimate[3];
      const auto* leg_controller = runner->debugLegController();
      for (int leg = 0; leg < 4; ++leg) {
        if (leg_controller) {
          const auto& data = leg_controller->datas[leg];
          const auto& cmd = leg_controller->commands[leg];
          WriteVec3(data.p);
          WriteVec3(data.v);
          WriteVec3(cmd.pDes);
          WriteVec3(cmd.vDes);
          WriteVec3(cmd.forceFeedForward);
        } else {
          WriteZeros(15);
        }
      }
    } else {
      WriteZeros(3 + 3 + 3 + 4 + 4 * 15);
    }
    const auto* cmpc = controller ? controller->debugConvexMPC() : nullptr;
    if (cmpc && cmpc->valid) {
      out_ << ',' << cmpc->gaitNumber << ',' << cmpc->currentGait << ','
           << cmpc->gaitPhase << ',' << cmpc->iterationCounter;
      WriteVec4(cmpc->contactStates);
      WriteVec4(cmpc->swingStates);
      WriteVec4(cmpc->seContactState);
      WriteVec4(cmpc->swingTimes);
      WriteVec4(cmpc->swingTimeRemaining);
      WriteVec4i(cmpc->mpcTableNow);
      WriteVec4i(cmpc->firstSwing);
      for (int leg = 0; leg < 4; ++leg) WriteVec3(cmpc->pFoot[leg]);
      for (int leg = 0; leg < 4; ++leg) WriteVec3(cmpc->pFootDes[leg]);
      for (int leg = 0; leg < 4; ++leg) WriteVec3(cmpc->vFootDes[leg]);
      for (int leg = 0; leg < 4; ++leg) WriteVec3(cmpc->FrDes[leg]);
    } else {
      WriteZeros(4 + 4 * 7 + 4 * 4 * 3);
    }
    out_ << '\n';
  }

 private:
  template <typename ArrayT>
  void WriteArray(const ArrayT& values) {
    for (const auto& v : values) out_ << ',' << static_cast<double>(v);
  }

  template <typename VecT>
  void WriteVec3(const VecT& v) {
    out_ << ',' << static_cast<double>(v[0]) << ','
         << static_cast<double>(v[1]) << ',' << static_cast<double>(v[2]);
  }

  template <typename VecT>
  void WriteVec4(const VecT& v) {
    out_ << ',' << static_cast<double>(v[0]) << ','
         << static_cast<double>(v[1]) << ',' << static_cast<double>(v[2])
         << ',' << static_cast<double>(v[3]);
  }

  void WriteVec4i(const Eigen::Vector4i& v) {
    out_ << ',' << v[0] << ',' << v[1] << ',' << v[2] << ',' << v[3];
  }

  void WriteZeros(int count) {
    for (int i = 0; i < count; ++i) out_ << ",0";
  }

  void AddArrayHeader(const char* prefix, int count) {
    for (int i = 0; i < count; ++i) out_ << ',' << prefix << i;
  }

  void AddLegVecHeader(const char* prefix) {
    for (int leg = 0; leg < 4; ++leg) {
      out_ << ',' << prefix << leg << "_x"
           << ',' << prefix << leg << "_y"
           << ',' << prefix << leg << "_z";
    }
  }

  void WriteHeader() {
    out_ << "t,mode,bridge_ready,bridge_reject_code,bridge_stage,"
            "alpha_q,alpha_qd,alpha_tau,"
            "requested_gait,cmpc_gait,control_mode,tilt,"
            "imu_roll,imu_pitch,imu_yaw,gyro_x,gyro_y,gyro_z,"
            "max_abs_tau_raw,max_abs_tau_pub,max_abs_dq_feedback,max_qerr";
    AddArrayHeader("q_feedback_", legbot::kNumJoints);
    AddArrayHeader("dq_feedback_", legbot::kNumJoints);
    AddArrayHeader("q_des_raw_", legbot::kNumJoints);
    AddArrayHeader("q_des_pub_", legbot::kNumJoints);
    AddArrayHeader("qd_des_raw_", legbot::kNumJoints);
    AddArrayHeader("qd_des_pub_", legbot::kNumJoints);
    AddArrayHeader("tau_raw_", legbot::kNumJoints);
    AddArrayHeader("tau_pub_", legbot::kNumJoints);
    AddArrayHeader("kp_", legbot::kNumJoints);
    AddArrayHeader("kd_", legbot::kNumJoints);
    out_ << ",state_rpy_x,state_rpy_y,state_rpy_z"
            ",state_pos_x,state_pos_y,state_pos_z"
            ",state_vworld_x,state_vworld_y,state_vworld_z"
            ",contact_0,contact_1,contact_2,contact_3";
    AddLegVecHeader("foot_p_leg");
    AddLegVecHeader("foot_v_leg");
    AddLegVecHeader("foot_pdes_leg");
    AddLegVecHeader("foot_vdes_leg");
    AddLegVecHeader("forceff_leg");
    out_ << ",cmpc_gait_number,cmpc_current_gait,cmpc_gait_phase,"
            "cmpc_iteration";
    AddArrayHeader("cmpc_desired_contact_", 4);
    AddArrayHeader("cmpc_swing_phase_", 4);
    AddArrayHeader("cmpc_se_contact_phase_", 4);
    AddArrayHeader("cmpc_swing_time_", 4);
    AddArrayHeader("cmpc_swing_time_remaining_", 4);
    AddArrayHeader("cmpc_mpc_table_now_", 4);
    AddArrayHeader("cmpc_first_swing_", 4);
    AddLegVecHeader("cmpc_foot_p_leg");
    AddLegVecHeader("cmpc_foot_pdes_leg");
    AddLegVecHeader("cmpc_foot_vdes_leg");
    AddLegVecHeader("cmpc_frdes_leg");
    out_ << '\n';
  }

  bool enabled_ = false;
  double period_s_ = 0.02;
  legbot::Clock::time_point t0_;
  legbot::Clock::time_point next_log_t_;
  std::ofstream out_;
};

int ReadKeyNonBlocking() {
  fd_set readfds;
  FD_ZERO(&readfds);
  FD_SET(STDIN_FILENO, &readfds);
  timeval timeout{};
  timeout.tv_sec = 0;
  timeout.tv_usec = 0;
  if (select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &timeout) <= 0) {
    return -1;
  }
  unsigned char c = 0;
  return read(STDIN_FILENO, &c, 1) == 1 ? static_cast<int>(c) : -1;
}

legbot::RuntimeSafetyResult CheckTickAdvances(
    const legbot::LowStateSnapshot& snapshot, const legbot::BridgeArgs& args,
    legbot::Clock::time_point now, uint32_t* last_tick,
    legbot::Clock::time_point* last_tick_change_t) {
  if (*last_tick == std::numeric_limits<uint32_t>::max() ||
      snapshot.tick != *last_tick) {
    *last_tick = snapshot.tick;
    *last_tick_change_t = now;
    return {};
  }
  if (legbot::SecondsSince(*last_tick_change_t, now) >
      args.lowstate_timeout_s) {
    return {false, "lowstate_tick_stale", -1, 0.};
  }
  return {};
}

double MaxKp(const legbot::DdsCommandArrays& command) {
  double out = 0.;
  for (double kp : command.kp) out = std::max(out, kp);
  return out;
}

double MaxKd(const legbot::DdsCommandArrays& command) {
  double out = 0.;
  for (double kd : command.kd) out = std::max(out, kd);
  return out;
}

legbot::RuntimeSafetyResult CheckInteractiveLowState(
    const legbot::LowStateSnapshot& snapshot,
    const legbot::LegBotDDSStateAdapter& state_adapter,
    const legbot::BridgeArgs& args, legbot::LoopStats* stats) {
  std::string lost_joint;
  if (state_adapter.HasLostMotor(snapshot.msg, &lost_joint)) {
    return {false, "motor_lost:" + lost_joint, -1, 0.};
  }
  if (!legbot::IsFiniteArray(snapshot.dds_q) ||
      !legbot::IsFiniteArray(snapshot.dds_dq)) {
    return {false, "lowstate_q_or_dq_nonfinite", -1, 0.};
  }
  const auto fb_limit = legbot::CheckFeedbackJointLimits(snapshot, args);
  if (!fb_limit.ok) return fb_limit;
  for (float q : snapshot.imu_quat_wxyz) {
    if (!std::isfinite(q)) return {false, "imu_quaternion_nonfinite", -1, q};
  }
  const double qw = snapshot.imu_quat_wxyz[0];
  const double qx = snapshot.imu_quat_wxyz[1];
  const double qy = snapshot.imu_quat_wxyz[2];
  const double qz = snapshot.imu_quat_wxyz[3];
  const double qnorm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
  if (qnorm < 0.5 || qnorm > 1.5) {
    return {false, "imu_quaternion_norm", -1, qnorm};
  }
  for (double g : snapshot.gyro) {
    if (!std::isfinite(g)) return {false, "gyro_nonfinite", -1, g};
    if (std::abs(g) > args.max_gyro_rad_s) {
      return {false, "max_gyro_rad_s", -1, std::abs(g)};
    }
  }
  const double tilt = state_adapter.Tilt(snapshot);
  if (stats) stats->max_tilt = std::max(stats->max_tilt, tilt);
  if (tilt > args.max_tilt_rad) return {false, "max_tilt_rad", -1, tilt};
  return {};
}

legbot::RuntimeSafetyResult CheckQErrorSoft(
    const legbot::LowStateSnapshot& snapshot,
    const legbot::DdsCommandArrays& command, const legbot::BridgeArgs& args,
    legbot::LoopStats* stats) {
  const auto model_q_des = legbot::DdsCommandModelQ(command);
  int qerr_joint = 0;
  const double qerr =
      legbot::MaxAbsError(snapshot.model_q, model_q_des, &qerr_joint);
  if (stats) stats->max_qerr = std::max(stats->max_qerr, qerr);
  if (args.max_q_error > 0. && qerr > args.max_q_error) {
    return {false,
            std::string("max_q_error:") +
                legbot::JointNameDds(legbot::ModelToDdsJoint(qerr_joint)),
            qerr_joint, qerr};
  }
  return {};
}

void PublishCurrentQDampingHoldThenDisable(
    const legbot::BridgeArgs& args, const legbot::LowStateSnapshot& snapshot,
    legbot::LowCmdPublisher* lowcmd_pub,
    legbot::LegBotDDSCommandAdapter* command_adapter, legbot::LoopStats* stats,
    const legbot::Clock::time_point& stats_t0) {
  const double hold_s = legbot::Clamp(args.hard_fault_damping_hold_s, 0.2, 0.5);
  const double period_s = 1. / args.cmd_hz;
  const auto start = legbot::Clock::now();
  auto next = start;
  std::cout << "[LEGBOT-DDS][HARD-FAULT] current-q damping hold "
            << std::fixed << std::setprecision(2) << hold_s
            << "s, then disable. No prone ramp.\n";
  while (g_running && legbot::SecondsSince(start, legbot::Clock::now()) < hold_s) {
    const auto command = command_adapter->MakeQOnlyCommand(
        snapshot.dds_q, 0.0, std::max(0.5, args.standup_kd));
    if (!command_adapter->PublishLowCmd(lowcmd_pub, command, stats, stats_t0)) {
      break;
    }
    next += std::chrono::duration_cast<legbot::Clock::duration>(
        std::chrono::duration<double>(period_s));
    std::this_thread::sleep_until(next);
  }
  command_adapter->DisableBurst(lowcmd_pub, args, stats, stats_t0);
}

void PrintDirectAlgoLog(const char* tag, const char* phase,
                        double phase_elapsed_s, double pre_trot_seconds,
                        double vx_cmd, double requested_gait,
                        const MIT_UserParameters* user_params,
                        const legbot::LowStateSnapshot& snapshot,
                        const legbot::LegBotDDSStateAdapter& state_adapter,
                        const legbot::OutputGuardStats& guard,
                        const RobotRunner* runner,
                        const MIT_Controller* controller) {
  std::cout << "[LEGBOT-DDS][" << tag << "]\n";
  if (phase) {
    std::cout << "phase=" << phase << " elapsed=" << std::fixed
              << std::setprecision(3) << phase_elapsed_s
              << " pre_trot_seconds=" << pre_trot_seconds << "\n";
  }
  std::cout << "requested_gait=" << requested_gait
            << " cmpc_gait=" << (user_params ? user_params->cmpc_gait : -1.0)
            << "\n";
  double max_abs_dq_feedback = 0.;
  for (double dq : snapshot.dds_dq) {
    max_abs_dq_feedback = std::max(max_abs_dq_feedback, std::abs(dq));
  }
  const double tilt = state_adapter.Tilt(snapshot);
  std::cout << "desired_vx=" << std::fixed << std::setprecision(4) << vx_cmd
            << " desired_vy=0.0000 desired_yaw_rate=0.0000\n";
  std::cout << "max_kp=" << MaxKp(guard.pub) << "\n"
            << "max_kd=" << MaxKd(guard.pub) << "\n"
            << "max_abs_tau_raw=" << guard.max_abs_tau_raw << "\n"
            << "max_abs_tau_pub=" << guard.max_abs_tau_pub << "\n"
            << "max_qdes_delta=" << guard.max_qdes_delta << "\n"
            << "max_abs_dq_feedback=" << max_abs_dq_feedback << "\n"
            << "num_qdes_clamped=" << guard.num_qdes_clamped << "\n"
            << "num_qdes_delta_clamped=" << guard.num_qdes_delta_clamped
            << "\n"
            << "num_qd_delta_clamped=" << guard.num_qd_delta_clamped
            << "\n"
            << "num_tau_clamped=" << guard.num_tau_clamped << "\n"
            << "q_feedback=" << legbot::ArrayText(snapshot.dds_q, 4) << "\n"
            << "dq_feedback=" << legbot::ArrayText(snapshot.dds_dq, 4) << "\n"
            << "q_des_raw=" << legbot::ArrayText(guard.raw.q, 4) << "\n"
            << "qd_des_raw=" << legbot::ArrayText(guard.raw.dq, 4) << "\n"
            << "q_des_pub=" << legbot::ArrayText(guard.pub.q, 4) << "\n"
            << "qd_des_pub=" << legbot::ArrayText(guard.pub.dq, 4) << "\n"
            << "tau_raw=" << legbot::ArrayText(guard.raw.tau, 4) << "\n"
            << "tau_pub=" << legbot::ArrayText(guard.pub.tau, 4) << "\n"
            << "kp_raw=" << legbot::ArrayText(guard.raw.kp, 3) << "\n"
            << "kp_pub=" << legbot::ArrayText(guard.pub.kp, 3) << "\n"
            << "kd_raw=" << legbot::ArrayText(guard.raw.kd, 3) << "\n"
            << "kd_pub=" << legbot::ArrayText(guard.pub.kd, 3) << "\n"
            << "tilt=" << std::fixed << std::setprecision(5) << tilt << "\n"
            << "max_tilt=" << tilt << "\n"
            << "gyro=" << legbot::ArrayText(snapshot.gyro, 5) << "\n";
  if (!runner) {
    std::cout << "state_est=unavailable\nleg_controller=unavailable\n";
    return;
  }
  const auto& se = runner->debugStateEstimate();
  std::cout << "state_est rpy=" << legbot::ArrayText(Vec3TextArray(se.rpy), 5)
            << " position=" << legbot::ArrayText(Vec3TextArray(se.position), 5)
            << " vWorld=" << legbot::ArrayText(Vec3TextArray(se.vWorld), 5)
            << " contactEstimate="
            << legbot::ArrayText(std::array<double, 4>{
                   static_cast<double>(se.contactEstimate[0]),
                   static_cast<double>(se.contactEstimate[1]),
                   static_cast<double>(se.contactEstimate[2]),
                   static_cast<double>(se.contactEstimate[3])},
                               3)
            << "\n";
  PrintConvexMPCDebug(controller);
  const auto* leg_controller = runner->debugLegController();
  if (!leg_controller) {
    std::cout << "leg_controller=unavailable\n";
    return;
  }
  for (int leg = 0; leg < 4; ++leg) {
    const auto& data = leg_controller->datas[leg];
    const auto& cmd = leg_controller->commands[leg];
    std::cout << "leg_io model_leg=" << leg
              << " p=" << legbot::ArrayText(Vec3TextArray(data.p), 5)
              << " v=" << legbot::ArrayText(Vec3TextArray(data.v), 5)
              << " q=" << legbot::ArrayText(Vec3TextArray(data.q), 5)
              << " qd=" << legbot::ArrayText(Vec3TextArray(data.qd), 5)
              << " pDes=" << legbot::ArrayText(Vec3TextArray(cmd.pDes), 5)
              << " vDes=" << legbot::ArrayText(Vec3TextArray(cmd.vDes), 5)
              << " qDes=" << legbot::ArrayText(Vec3TextArray(cmd.qDes), 5)
              << " tauFF=" << legbot::ArrayText(Vec3TextArray(cmd.tauFeedForward), 5)
              << " forceFF=" << legbot::ArrayText(Vec3TextArray(cmd.forceFeedForward), 5)
              << "\n";
  }
}

void PrintMitOutputWaitLog(const char* tag, double wait_s, double vx_cmd,
                           double requested_gait,
                           const legbot::LowStateSnapshot& snapshot,
                           const legbot::LegBotDDSStateAdapter& state_adapter,
                           const legbot::DdsCommandArrays& raw,
                           const MitOutputReadiness& readiness,
                           const MIT_Controller* controller,
                           const RobotRunner* runner,
                           const RobotControlParameters* robot_params,
                           const MIT_UserParameters* user_params,
                           const GamepadCommand* gamepad_command) {
  std::cout << "[LEGBOT-DDS][" << tag << "] t=" << std::fixed
            << std::setprecision(2) << wait_s
            << " publish=Q-STAND-HOLD ready="
            << (readiness.ready ? "yes" : "no")
            << " reason=" << readiness.reason << "\n";
  if (std::strcmp(tag, "MIT-FORWARD-OUTPUT-WAIT") == 0) {
    std::cout << "mode=LOCOMOTION_PRE_TROT\n";
  }
  std::cout << "control_mode="
            << (robot_params ? robot_params->control_mode : -1.0)
            << " requested_gait=" << requested_gait << " cmpc_gait="
            << (user_params ? user_params->cmpc_gait : -1.0)
            << " desired_vx="
            << (gamepad_command ? gamepad_command->leftStickAnalog[1] : 0.f)
            << " desired_vy="
            << (gamepad_command ? gamepad_command->leftStickAnalog[0] : 0.f)
            << " desired_yaw_rate="
            << (gamepad_command ? gamepad_command->rightStickAnalog[0] : 0.f)
            << "\n";
  const auto* fsm = controller ? controller->debugControlFSM() : nullptr;
  if (fsm && fsm->currentState) {
    std::cout << "fsm_current=" << FsmStateNameText(fsm->currentState->stateName)
              << " fsm_current_next="
              << FsmStateNameText(fsm->currentState->nextStateName)
              << " fsm_next="
              << FsmStateNameText(fsm->nextStateName) << "\n";
  } else {
    std::cout << "fsm_current=unavailable fsm_next=unavailable\n";
  }
  if (vx_cmd != 0.) {
    std::cout << "vx_cmd=" << std::fixed << std::setprecision(4) << vx_cmd
              << "\n";
  }
  PrintConvexMPCDebug(controller);
  std::cout << "max_kp_raw=" << readiness.max_kp << "\n"
            << "max_kd_raw=" << readiness.max_kd << "\n"
            << "max_abs_q_des_raw=" << readiness.max_abs_q << "\n"
            << "q_feedback=" << legbot::ArrayText(snapshot.dds_q, 4) << "\n"
            << "dq_feedback=" << legbot::ArrayText(snapshot.dds_dq, 4) << "\n"
            << "q_des_raw=" << legbot::ArrayText(raw.q, 4) << "\n"
            << "tau_raw=" << legbot::ArrayText(raw.tau, 4) << "\n"
            << "kp_raw=" << legbot::ArrayText(raw.kp, 3) << "\n"
            << "kd_raw=" << legbot::ArrayText(raw.kd, 3) << "\n"
            << "tilt=" << std::fixed << std::setprecision(5)
            << state_adapter.Tilt(snapshot) << "\n"
            << "gyro=" << legbot::ArrayText(snapshot.gyro, 5) << "\n";
  const auto* leg_controller = runner ? runner->debugLegController() : nullptr;
  if (!leg_controller) {
    std::cout << "leg_controller=unavailable\n";
    return;
  }
  for (int leg = 0; leg < 4; ++leg) {
    const auto& cmd = leg_controller->commands[leg];
    std::cout << "leg_cmd model_leg=" << leg
              << " qDes=" << legbot::ArrayText(Vec3TextArray(cmd.qDes), 4)
              << " qdDes=" << legbot::ArrayText(Vec3TextArray(cmd.qdDes), 4)
              << " pDes=" << legbot::ArrayText(Vec3TextArray(cmd.pDes), 4)
              << " vDes=" << legbot::ArrayText(Vec3TextArray(cmd.vDes), 4)
              << " kpJointDiag="
              << legbot::ArrayText(Mat3DiagTextArray(cmd.kpJoint), 3)
              << " kdJointDiag="
              << legbot::ArrayText(Mat3DiagTextArray(cmd.kdJoint), 3)
              << " kpCartDiag="
              << legbot::ArrayText(Mat3DiagTextArray(cmd.kpCartesian), 3)
              << " kdCartDiag="
              << legbot::ArrayText(Mat3DiagTextArray(cmd.kdCartesian), 3)
              << " tauFF="
              << legbot::ArrayText(Vec3TextArray(cmd.tauFeedForward), 4)
              << " forceFF="
              << legbot::ArrayText(Vec3TextArray(cmd.forceFeedForward), 4)
              << "\n";
  }
}

void RunGuardedStandup(const legbot::BridgeArgs& args,
                       legbot::LegBotDDSStateAdapter* state_adapter,
                       legbot::LowCmdPublisher* lowcmd_pub,
                       legbot::LegBotDDSCommandAdapter* command_adapter,
                       legbot::LoopStats* stats,
                       const legbot::Clock::time_point& stats_t0) {
  const auto q_start = state_adapter->Snapshot().dds_q;
  const auto q_stand = legbot::StandQDds();
  const double period_s = 1. / args.cmd_hz;
  auto next = legbot::Clock::now();
  auto next_print = next;
  auto last_sample_t = next;
  auto last_tick_change_t = next;
  uint32_t last_lowstate_tick = std::numeric_limits<uint32_t>::max();
  uint32_t last_recorded_tick = std::numeric_limits<uint32_t>::max();

  auto run_phase = [&](const char* phase, double phase_s,
                       const std::array<double, legbot::kNumJoints>& target0,
                       const std::array<double, legbot::kNumJoints>& target1) {
    const auto phase_start = legbot::Clock::now();
    while (g_running) {
      const auto now = legbot::Clock::now();
      const double elapsed = legbot::SecondsSince(phase_start, now);
      if (elapsed >= phase_s) break;
      if (state_adapter->IsTimeout()) {
        std::cerr << "[LEGBOT-DDS][STANDUP] rt_lowstate_timeout during "
                  << phase << "\n";
        return;
      }
      const auto snapshot = state_adapter->Snapshot();
      const auto tick_safety = CheckTickAdvances(
          snapshot, args, now, &last_lowstate_tick, &last_tick_change_t);
      if (!tick_safety.ok) {
        std::cerr << "[LEGBOT-DDS][STANDUP] " << tick_safety.reason << "\n";
        return;
      }
      legbot::RecordLowStateTiming(snapshot.tick, stats, &last_sample_t,
                                 &last_recorded_tick);
      const auto lowstate_safety =
          CheckInteractiveLowState(snapshot, *state_adapter, args, stats);
      if (!lowstate_safety.ok) {
        std::cerr << "[LEGBOT-DDS][STANDUP] safety=" << lowstate_safety.reason
                  << " value=" << lowstate_safety.value << "\n";
        return;
      }

      std::array<double, legbot::kNumJoints> q_target{};
      const double blend_s =
          phase_s > 1.e-9 ? legbot::SmoothStep(elapsed / phase_s) : 1.;
      for (std::size_t i = 0; i < legbot::kNumJoints; ++i) {
        q_target[i] = (1. - blend_s) * target0[i] + blend_s * target1[i];
      }
      const auto command = command_adapter->MakeQOnlyCommand(
          q_target, args.standup_kp, args.standup_kd);
      const auto qerr_safety = CheckQErrorSoft(snapshot, command, args, stats);
      if (!qerr_safety.ok) {
        std::cerr << "[LEGBOT-DDS][STANDUP] safety=" << qerr_safety.reason
                  << " value=" << qerr_safety.value << "\n";
        return;
      }
      if (!command_adapter->PublishLowCmd(lowcmd_pub, command, stats,
                                          stats_t0)) {
        std::cerr << "[LEGBOT-DDS][STANDUP] publish_failed during " << phase
                  << "\n";
        return;
      }

      if (now >= next_print) {
        std::cout << "[LEGBOT-DDS][STANDUP] phase=" << phase << " t="
                  << std::fixed << std::setprecision(2) << elapsed
                  << " q_target=" << legbot::ArrayText(q_target, 4) << "\n";
        next_print = now + std::chrono::seconds(1);
      }
      ++stats->loop_count;
      next += std::chrono::duration_cast<legbot::Clock::duration>(
          std::chrono::duration<double>(period_s));
      std::this_thread::sleep_until(next);
    }
  };

  std::cout << "[LEGBOT-DDS] Guarded stand-up: current q -> stand q for "
            << args.standup_ramp_seconds
            << "s; controller standing prehold "
            << args.standup_prehold_seconds << "s\n";
  run_phase("standup_ramp", args.standup_ramp_seconds, q_start, q_stand);
}

std::string RunJointNudgeTest(const legbot::BridgeArgs& args,
                              legbot::LegBotDDSStateAdapter* state_adapter,
                              legbot::LowCmdPublisher* lowcmd_pub,
                              legbot::LegBotDDSCommandAdapter* command_adapter,
                              legbot::LoopStats* stats,
                              const legbot::Clock::time_point& stats_t0) {
  if (!lowcmd_pub) return "joint_nudge_missing_publisher";
  const auto start_snapshot = state_adapter->Snapshot();
  auto q_nudge = start_snapshot.dds_q;
  q_nudge[static_cast<std::size_t>(args.nudge_joint)] += args.nudge_delta;
  const double nudge_kp = std::max(args.standup_kp, 20.0);
  const double nudge_kd = std::max(args.standup_kd, 3.0);
  const double period_s = 1. / args.cmd_hz;
  auto next = legbot::Clock::now();
  auto next_print = next;
  auto last_sample_t = next;
  auto last_tick_change_t = next;
  uint32_t last_lowstate_tick = std::numeric_limits<uint32_t>::max();
  uint32_t last_recorded_tick = std::numeric_limits<uint32_t>::max();
  const auto phase_start = next;

  std::cout << "[LEGBOT-DDS][JOINT-NUDGE] start joint="
            << legbot::JointNameDds(args.nudge_joint)
            << " index=" << args.nudge_joint
            << " delta=" << args.nudge_delta
            << " seconds=" << args.nudge_seconds
            << " kp=" << nudge_kp
            << " kd=" << nudge_kd
            << " q_start=" << legbot::ArrayText(start_snapshot.dds_q, 5)
            << " q_nudge=" << legbot::ArrayText(q_nudge, 5) << "\n";

  while (g_running) {
    const auto now = legbot::Clock::now();
    const double elapsed = legbot::SecondsSince(phase_start, now);
    if (elapsed >= args.nudge_seconds) break;
    if (state_adapter->IsTimeout()) {
      return "joint_nudge_failed:rt_lowstate_timeout";
    }
    const auto snapshot = state_adapter->Snapshot();
    const auto tick_safety = CheckTickAdvances(
        snapshot, args, now, &last_lowstate_tick, &last_tick_change_t);
    if (!tick_safety.ok) {
      return std::string("joint_nudge_failed:") + tick_safety.reason;
    }
    legbot::RecordLowStateTiming(snapshot.tick, stats, &last_sample_t,
                               &last_recorded_tick);
    const auto lowstate_safety =
        CheckInteractiveLowState(snapshot, *state_adapter, args, stats);
    if (!lowstate_safety.ok) {
      return std::string("joint_nudge_failed:") + lowstate_safety.reason;
    }
    auto command =
        command_adapter->MakeQOnlyCommand(q_nudge, nudge_kp, nudge_kd);
    for (double& tau : command.tau) tau = 0.;
    if (!command_adapter->PublishLowCmd(lowcmd_pub, command, stats,
                                        stats_t0)) {
      return "publish_failed:joint_nudge";
    }
    if (now >= next_print) {
      std::cout << "[LEGBOT-DDS][JOINT-NUDGE] t=" << std::fixed
                << std::setprecision(3) << elapsed
                << " joint=" << legbot::JointNameDds(args.nudge_joint)
                << " q_feedback="
                << snapshot.dds_q[static_cast<std::size_t>(args.nudge_joint)]
                << " q_des="
                << q_nudge[static_cast<std::size_t>(args.nudge_joint)]
                << " full_q_feedback=" << legbot::ArrayText(snapshot.dds_q, 5)
                << "\n";
      next_print = now + std::chrono::duration_cast<legbot::Clock::duration>(
                             std::chrono::duration<double>(0.1));
    }
    ++stats->loop_count;
    next += std::chrono::duration_cast<legbot::Clock::duration>(
        std::chrono::duration<double>(period_s));
    std::this_thread::sleep_until(next);
  }

  const auto stand_command = command_adapter->MakeQOnlyCommand(
      legbot::StandQDds(), args.standup_kp, args.standup_kd);
  for (int i = 0; i < 20 && g_running; ++i) {
    if (!command_adapter->PublishLowCmd(lowcmd_pub, stand_command, stats,
                                        stats_t0)) {
      return "publish_failed:joint_nudge_q_stand_hold";
    }
    std::this_thread::sleep_for(std::chrono::duration<double>(period_s));
  }
  std::cout << "[LEGBOT-DDS][JOINT-NUDGE] done publish=Q-STAND-HOLD\n";
  return "joint_nudge_completed";
}

void RunGuardedRampDown(const legbot::BridgeArgs& args,
                        const std::array<double, legbot::kNumJoints>& prone_q,
                        legbot::LegBotDDSStateAdapter* state_adapter,
                        legbot::LowCmdPublisher* lowcmd_pub,
                        legbot::LegBotDDSCommandAdapter* command_adapter,
                        legbot::LoopStats* stats,
                        const legbot::Clock::time_point& stats_t0) {
  if (args.shutdown_ramp_seconds <= 0. &&
      args.shutdown_prehold_seconds <= 0.) {
    return;
  }

  const auto q_start = state_adapter->Snapshot().dds_q;
  const double period_s = 1. / args.cmd_hz;
  auto next = legbot::Clock::now();
  auto next_print = next;
  auto last_sample_t = next;
  auto last_tick_change_t = next;
  uint32_t last_lowstate_tick = std::numeric_limits<uint32_t>::max();
  uint32_t last_recorded_tick = std::numeric_limits<uint32_t>::max();

  auto run_phase = [&](const char* phase, double phase_s,
                       const std::array<double, legbot::kNumJoints>& target0,
                       const std::array<double, legbot::kNumJoints>& target1) {
    const auto phase_start = legbot::Clock::now();
    while (g_running) {
      const auto now = legbot::Clock::now();
      const double elapsed = legbot::SecondsSince(phase_start, now);
      if (elapsed >= phase_s) break;
      if (state_adapter->IsTimeout()) {
        std::cerr << "[LEGBOT-DDS][SHUTDOWN] rt_lowstate_timeout during "
                  << phase << "\n";
        return;
      }
      const auto snapshot = state_adapter->Snapshot();
      const auto tick_safety = CheckTickAdvances(
          snapshot, args, now, &last_lowstate_tick, &last_tick_change_t);
      if (!tick_safety.ok) {
        std::cerr << "[LEGBOT-DDS][SHUTDOWN] " << tick_safety.reason << "\n";
        return;
      }
      legbot::RecordLowStateTiming(snapshot.tick, stats, &last_sample_t,
                                 &last_recorded_tick);
      const auto lowstate_safety =
          CheckInteractiveLowState(snapshot, *state_adapter, args, stats);
      if (!lowstate_safety.ok) {
        std::cerr << "[LEGBOT-DDS][SHUTDOWN] safety=" << lowstate_safety.reason
                  << " value=" << lowstate_safety.value << "\n";
        return;
      }

      std::array<double, legbot::kNumJoints> q_target{};
      const double blend_s =
          phase_s > 1.e-9 ? legbot::SmoothStep(elapsed / phase_s) : 1.;
      for (std::size_t i = 0; i < legbot::kNumJoints; ++i) {
        q_target[i] = (1. - blend_s) * target0[i] + blend_s * target1[i];
      }
      const auto command = command_adapter->MakeQOnlyCommand(
          q_target, args.standup_kp, args.standup_kd);
      const auto qerr_safety = CheckQErrorSoft(snapshot, command, args, stats);
      if (!qerr_safety.ok) {
        std::cerr << "[LEGBOT-DDS][SHUTDOWN] safety=" << qerr_safety.reason
                  << " value=" << qerr_safety.value << "\n";
        return;
      }
      if (!command_adapter->PublishLowCmd(lowcmd_pub, command, stats,
                                          stats_t0)) {
        std::cerr << "[LEGBOT-DDS][SHUTDOWN] publish_failed during " << phase
                  << "\n";
        return;
      }

      if (now >= next_print) {
        std::cout << "[LEGBOT-DDS][SHUTDOWN] phase=" << phase << " t="
                  << std::fixed << std::setprecision(2) << elapsed
                  << " q_target=" << legbot::ArrayText(q_target, 4) << "\n";
        next_print = now + std::chrono::seconds(1);
      }
      ++stats->loop_count;
      next += std::chrono::duration_cast<legbot::Clock::duration>(
          std::chrono::duration<double>(period_s));
      std::this_thread::sleep_until(next);
    }
  };

  std::cout << "[LEGBOT-DDS] Guarded shutdown: current q -> startup prone q for "
            << args.shutdown_ramp_seconds << "s, prehold "
            << args.shutdown_prehold_seconds << "s before disable\n";
  run_phase("shutdown_ramp", args.shutdown_ramp_seconds, q_start, prone_q);
  run_phase("shutdown_prehold", args.shutdown_prehold_seconds, prone_q,
            prone_q);
}

bool BootstrapMotorFeedback(const legbot::BridgeArgs& args,
                            legbot::LegBotDDSStateAdapter* state_adapter,
                            legbot::LowCmdPublisher* lowcmd_pub,
                            legbot::LegBotDDSCommandAdapter* command_adapter,
                            legbot::LoopStats* stats,
                            const legbot::Clock::time_point& stats_t0) {
  const auto start = legbot::Clock::now();
  auto next = start;
  auto last_sample_t = start;
  uint32_t last_recorded_tick = std::numeric_limits<uint32_t>::max();
  const double period_s = 1. / args.cmd_hz;

  std::cout << "[LEGBOT-DDS] Bootstrapping motor feedback with q-hold LowCmd; "
               "waiting for lost=none.\n";
  while (g_running) {
    const auto now = legbot::Clock::now();
    if (legbot::SecondsSince(start, now) > args.wait_lowstate_s) {
      std::cerr << "[LEGBOT-DDS][ERROR] motor feedback is still lost after bootstrap\n";
      command_adapter->DisableBurst(lowcmd_pub, args, stats, stats_t0);
      return false;
    }
    if (state_adapter->IsTimeout()) {
      std::cerr << "[LEGBOT-DDS][ERROR] rt_lowstate_timeout during feedback bootstrap\n";
      command_adapter->DisableBurst(lowcmd_pub, args, stats, stats_t0);
      return false;
    }

    const auto snapshot = state_adapter->Snapshot();
    legbot::RecordLowStateTiming(snapshot.tick, stats, &last_sample_t,
                               &last_recorded_tick);
    if (!state_adapter->HasLostMotor(snapshot.msg, nullptr)) {
      std::cout << "[LEGBOT-DDS] Motor feedback valid after bootstrap.\n";
      return true;
    }

    const auto command = command_adapter->MakeQOnlyCommand(
        snapshot.dds_q, 0.0, std::max(0.5, args.standup_kd));
    if (!command_adapter->PublishLowCmd(lowcmd_pub, command, stats,
                                        stats_t0)) {
      std::cerr << "[LEGBOT-DDS][ERROR] publish_failed during feedback bootstrap\n";
      command_adapter->DisableBurst(lowcmd_pub, args, stats, stats_t0);
      return false;
    }
    ++stats->loop_count;
    next += std::chrono::duration_cast<legbot::Clock::duration>(
        std::chrono::duration<double>(period_s));
    std::this_thread::sleep_until(next);
  }
  return false;
}

std::string RunInteractiveControl(const legbot::BridgeArgs& args,
                                  MIT_Controller* controller,
                                  RobotRunner* runner,
                                  RobotControlParameters* robot_params,
                                  MIT_UserParameters* user_params,
                                  GamepadCommand* gamepad_command,
                                  SpiData* spi_data,
                                  SpiCommand* spi_command,
                                  VectorNavData* vector_nav_data,
                                  CheaterState<double>* cheater_state,
                                  legbot::LegBotDDSStateAdapter* state_adapter,
                                  legbot::LowCmdPublisher* lowcmd_pub,
                                  legbot::LegBotDDSCommandAdapter* command_adapter,
                                  legbot::LoopStats* stats,
                                  const legbot::Clock::time_point& stats_t0) {
  enum class Mode {
    kQHold,
    kMitStandArming,
    kMitStandOutputWait,
    kMitStandDirect,
    kKey3Locomotion,
  };

  TerminalRawMode raw_mode;
  Mode mode = Mode::kQHold;
  bool standing_ready = false;
  auto mit_arm_start = legbot::Clock::now();
  auto mit_output_wait_start = legbot::Clock::now();
  std::array<double, legbot::kNumJoints> previous_q_des = legbot::StandQDds();
  std::array<double, legbot::kNumJoints> previous_qd_des{};
  bool has_previous_q_des = false;
  auto key3_phase_start = legbot::Clock::now();
  auto key3_stable_start = legbot::Clock::now();
  bool key3_has_stable_start = false;
  bool key3_timeout_logged = false;
  bool key3_gait4_foot_jump_diag_logged = false;
  bool key3_gait0_entry_logged = false;
  std::array<double, legbot::kNumJoints> gait0_entry_q_fb{};
  std::array<double, legbot::kNumJoints> gait0_entry_q_hold{};
  bool gait0_has_previous_raw_q = false;
  std::array<double, legbot::kNumJoints> gait0_previous_raw_q{};
  int gait0_raw_stable_count = 0;
  bool gait0_raw_released = false;
  auto gait0_window_start = legbot::Clock::now();
  int gait0_window_mpc_table_transitions = 0;
  bool gait0_window_has_mpc_table = false;
  std::array<double, 4> gait0_window_last_mpc_table{};
  double gait0_window_max_raw_q_step = 0.;
  bool gait0_window_bad_raw_q_step = false;
  double gait0_previous_entry_violation = std::numeric_limits<double>::quiet_NaN();
  int gait0_entry_violation_increase_count = 0;
  bool gait0_entry_reference_settled = false;
  Gait0MpcTrace gait0_previous_mpc_trace;
  int gait0_monitor_qdes_clamped_count = 0;
  int gait0_monitor_entry_violation_count = 0;
  double gait0_blend_alpha_q = 0.;
  std::array<double, legbot::kNumJoints> gait0_blend_start_q{};
  std::array<double, legbot::kNumJoints> gait0_blend_start_dq{};
  std::array<double, legbot::kNumJoints> gait0_blend_start_tau{};
  std::array<double, legbot::kNumJoints> key3_hold_q = legbot::StandQDds();
  legbot::DdsCommandArrays key3_last_safe_command{};
  Key3Phase key3_phase = Key3Phase::kNone;
  std::string last_direct_reject_reason;
  std::string last_gait4_to_gait0_wait_reason;

  auto set_key3_phase = [&](Key3Phase next_phase,
                            const char* reason = "transition",
                            bool ready = true,
                            BridgeRejectCode reject_code =
                                BridgeRejectCode::kOk) {
    const auto transition_now = legbot::Clock::now();
    const double elapsed_in_prev_phase =
        key3_phase == Key3Phase::kNone
            ? 0.0
            : legbot::SecondsSince(key3_phase_start, transition_now);
    key3_phase = next_phase;
    key3_phase_start = transition_now;
    key3_stable_start = key3_phase_start;
    key3_has_stable_start = false;
    key3_timeout_logged = false;
    key3_gait4_foot_jump_diag_logged = false;
    key3_gait0_entry_logged = false;
    if (next_phase == Key3Phase::kGait0Shadow) {
      gait0_has_previous_raw_q = false;
      gait0_previous_raw_q.fill(0.);
      gait0_raw_stable_count = 0;
      gait0_raw_released = false;
      gait0_window_start = transition_now;
      gait0_window_mpc_table_transitions = 0;
      gait0_window_has_mpc_table = false;
      gait0_window_last_mpc_table.fill(0.);
      gait0_window_max_raw_q_step = 0.;
      gait0_window_bad_raw_q_step = false;
      gait0_previous_entry_violation =
          std::numeric_limits<double>::quiet_NaN();
      gait0_entry_violation_increase_count = 0;
      gait0_entry_reference_settled = false;
      gait0_previous_mpc_trace = Gait0MpcTrace{};
      gait0_monitor_qdes_clamped_count = 0;
      gait0_monitor_entry_violation_count = 0;
      gait0_blend_alpha_q = 0.;
      gait0_blend_start_q.fill(0.);
      gait0_blend_start_dq.fill(0.);
      gait0_blend_start_tau.fill(0.);
    } else if (next_phase == Key3Phase::kGait0Blend) {
      gait0_blend_alpha_q = 0.;
      gait0_blend_start_q = key3_last_safe_command.q;
      gait0_blend_start_dq = key3_last_safe_command.dq;
      gait0_blend_start_tau = key3_last_safe_command.tau;
      gait0_monitor_qdes_clamped_count = 0;
      gait0_monitor_entry_violation_count = 0;
    }
    last_direct_reject_reason.clear();
    last_gait4_to_gait0_wait_reason.clear();
    std::cout << "[LEGBOT-DDS][KEY3-PHASE] phase="
              << Key3PhaseText(key3_phase)
              << " control_mode=" << robot_params->control_mode
              << " requested_gait="
              << ((key3_phase == Key3Phase::kGait0Shadow ||
                   key3_phase == Key3Phase::kGait0Blend ||
                   key3_phase == Key3Phase::kGait0Direct)
                      ? args.test_gait
                      : 4.0)
              << " cmpc_gait=" << user_params->cmpc_gait
              << " desired_vx=0 desired_vy=0 desired_yaw_rate=0"
              << " ready=" << (ready ? "yes" : "no")
              << " reject=" << BridgeRejectCodeText(reject_code)
              << " reason=" << reason
              << " elapsed_in_prev_phase=" << std::fixed
              << std::setprecision(3) << elapsed_in_prev_phase
              << " new_phase_elapsed=0\n";
  };

  auto next = legbot::Clock::now();
  auto next_print = next;
  auto last_sample_t = next;
  auto last_tick_change_t = next;
  uint32_t last_lowstate_tick = std::numeric_limits<uint32_t>::max();
  uint32_t last_recorded_tick = std::numeric_limits<uint32_t>::max();
  const double period_s = 1. / args.cmd_hz;
  CsvLogger csv_logger(args.csv_log_path, args.csv_hz, stats_t0);
  PrintLegBotMappingAndFkDiagnostics(state_adapter->Snapshot(),
                                   runner ? runner->debugLegController()
                                          : nullptr,
                                   "interactive_start");

  std::cout
      << "[LEGBOT-DDS] Interactive control ready.\n"
      << "  keys: 1 q-only stand-up, 2 MIT balance-stand direct, "
         "3 MIT forward direct, x q-stand hold, 4 down+disable+exit, "
         "q down+disable+exit\n";

  while (g_running) {
    const auto now = legbot::Clock::now();
    const int key = ReadKeyNonBlocking();
    if (key >= 0) {
      switch (key) {
        case '1':
          std::cout << "\n[LEGBOT-DDS][KEY] 1: q-only stand-up ramp\n";
          RunGuardedStandup(args, state_adapter, lowcmd_pub, command_adapter,
                            stats, stats_t0);
          standing_ready = true;
          mode = Mode::kQHold;
          key3_phase = Key3Phase::kNone;
          gait0_has_previous_raw_q = false;
          gait0_raw_stable_count = 0;
          gait0_raw_released = false;
          previous_q_des = legbot::StandQDds();
          previous_qd_des.fill(0.);
          has_previous_q_des = true;
          PrintLegBotMappingAndFkDiagnostics(state_adapter->Snapshot(),
                                           runner ? runner->debugLegController()
                                                  : nullptr,
                                           "after_q_stand_hold");
          next = legbot::Clock::now();
          next_print = next;
          break;
        case '2':
          if (!standing_ready) {
            std::cout << "\n[LEGBOT-DDS][KEY] press 1 before MIT balance-stand direct\n";
            break;
          }
          std::cout << "\n[LEGBOT-DDS][KEY] 2: arm MIT FSM, wait for valid SpiCommand, then BALANCE_STAND direct\n";
          mode = Mode::kMitStandArming;
          key3_phase = Key3Phase::kNone;
          gait0_has_previous_raw_q = false;
          gait0_raw_stable_count = 0;
          gait0_raw_released = false;
          mit_arm_start = legbot::Clock::now();
          robot_params->control_mode = kStandUpControlMode;
          previous_q_des = legbot::StandQDds();
          previous_qd_des.fill(0.);
          has_previous_q_des = true;
          next_print = legbot::Clock::now();
          break;
        case '3':
          if (!standing_ready) {
            std::cout << "\n[LEGBOT-DDS][KEY] press 1 before MIT forward direct\n";
            break;
          }
          {
            const auto fsm_state = CurrentFsmStateName(controller);
            const bool already_balance_stand =
                fsm_state == FSM_StateName::BALANCE_STAND;
            std::cout << "\n[LEGBOT-DDS][KEY] 3: key3 safe Locomotion takeover "
                      << "(arming -> gait4 shadow -> gait4 direct -> "
                         "gait0 shadow -> gait0 blend -> gait0 direct)"
                      << " requested_gait=" << args.test_gait
                      << " fsm_current=" << FsmStateNameText(fsm_state)
                      << "\n";
            mode = Mode::kKey3Locomotion;
            gait0_has_previous_raw_q = false;
            gait0_raw_stable_count = 0;
            gait0_raw_released = false;
            key3_hold_q = state_adapter->Snapshot().dds_q;
            key3_last_safe_command = command_adapter->MakeQOnlyCommand(
                key3_hold_q, args.standup_kp, args.standup_kd);
            PrintLegBotMappingAndFkDiagnostics(state_adapter->Snapshot(),
                                             runner ? runner->debugLegController()
                                                    : nullptr,
                                             "key3_entry");
            if (already_balance_stand) {
              robot_params->control_mode = kLocomotionControlMode;
              user_params->cmpc_gait =
                  4.0;
              set_key3_phase(Key3Phase::kLocomotionGait4Shadow,
                             "already_balance_stand");
            } else {
              mit_arm_start = legbot::Clock::now();
              robot_params->control_mode = kStandUpControlMode;
              user_params->cmpc_gait = 4.0;
              set_key3_phase(Key3Phase::kMitForwardArming,
                             "start_from_non_balance_stand");
            }
          }
          previous_q_des = key3_hold_q;
          previous_qd_des.fill(0.);
          has_previous_q_des = true;
          last_direct_reject_reason.clear();
          next_print = legbot::Clock::now();
          break;
        case 'x':
        case 'X':
          std::cout << "\n[LEGBOT-DDS][KEY] Q-STAND-HOLD\n";
          mode = Mode::kQHold;
          key3_phase = Key3Phase::kNone;
          gait0_has_previous_raw_q = false;
          gait0_raw_stable_count = 0;
          gait0_raw_released = false;
          previous_q_des = legbot::StandQDds();
          previous_qd_des.fill(0.);
          has_previous_q_des = true;
          PrintLegBotMappingAndFkDiagnostics(state_adapter->Snapshot(),
                                           runner ? runner->debugLegController()
                                                  : nullptr,
                                           "q_stand_hold_key");
          break;
        case '4':
        case 'q':
        case 'Q':
          std::cout << "\n[LEGBOT-DDS][KEY] down ramp + disable/exit\n";
          RunGuardedRampDown(args, legbot::DownQDds(), state_adapter,
                             lowcmd_pub, command_adapter, stats, stats_t0);
          command_adapter->DisableBurst(lowcmd_pub, args, stats, stats_t0);
          return "user_exit_down_disable";
        default:
          break;
      }
    }

    if (state_adapter->IsTimeout()) {
      const auto snapshot = state_adapter->Snapshot();
      std::cerr << "[LEGBOT-DDS][HARD-FAULT] rt_lowstate_timeout during interactive-control\n";
      PublishCurrentQDampingHoldThenDisable(args, snapshot, lowcmd_pub,
                                            command_adapter, stats, stats_t0);
      return "runtime_safety_failed:rt_lowstate_timeout";
    }
    const auto loop_t0 = legbot::Clock::now();
    const auto snapshot = state_adapter->Snapshot();
    const auto tick_safety = CheckTickAdvances(
        snapshot, args, now, &last_lowstate_tick, &last_tick_change_t);
    if (!tick_safety.ok) {
      std::cerr << "[LEGBOT-DDS][HARD-FAULT] " << tick_safety.reason << "\n";
      PublishCurrentQDampingHoldThenDisable(args, snapshot, lowcmd_pub,
                                            command_adapter, stats, stats_t0);
      return std::string("runtime_safety_failed:") + tick_safety.reason;
    }
    legbot::RecordLowStateTiming(snapshot.tick, stats, &last_sample_t,
                               &last_recorded_tick);
    const auto lowstate_safety =
        CheckInteractiveLowState(snapshot, *state_adapter, args, stats);
    if (!lowstate_safety.ok) {
      if (lowstate_safety.reason.find("runtime_joint_limit:") == 0) {
        const auto limit = legbot::FeedbackQLimitForModelJoint(
            legbot::DdsToModelJoint(lowstate_safety.joint), args);
        std::cerr << "[LEGBOT-DDS][JOINT-LIMIT-FAULT]\n"
                  << "joint=" << legbot::JointNameDds(lowstate_safety.joint)
                  << "\nindex=" << lowstate_safety.joint
                  << "\nq=" << lowstate_safety.value
                  << "\nlimit=[" << limit.first << "," << limit.second
                  << "]\n";
      } else {
        std::cerr << "[LEGBOT-DDS][HARD-FAULT] " << lowstate_safety.reason
                  << " value=" << lowstate_safety.value << "\n";
      }
      PublishCurrentQDampingHoldThenDisable(args, snapshot, lowcmd_pub,
                                            command_adapter, stats, stats_t0);
      return std::string("runtime_safety_failed:") + lowstate_safety.reason;
    }

    if (mode == Mode::kQHold) {
      const auto target = standing_ready ? legbot::StandQDds() : snapshot.dds_q;
      const auto command = command_adapter->MakeQOnlyCommand(
          target, standing_ready ? args.standup_kp : 0.0, args.standup_kd);
      const auto qerr_safety = CheckQErrorSoft(snapshot, command, args, stats);
      if (!qerr_safety.ok) {
        std::cerr << "[LEGBOT-DDS][SOFT-FAULT] " << qerr_safety.reason
                  << " value=" << qerr_safety.value
                  << "; staying in current-q damping hold\n";
      }
      if (!command_adapter->PublishLowCmd(lowcmd_pub, command, stats,
                                          stats_t0)) {
        std::cerr << "[LEGBOT-DDS][ERROR] publish_failed during interactive-qhold\n";
        command_adapter->DisableBurst(lowcmd_pub, args, stats, stats_t0);
        return "publish_failed:interactive-qhold";
      }
      if (now >= next_print) {
        PrintLowState(snapshot, *state_adapter, *stats,
                      legbot::SecondsSince(stats_t0, now));
        PrintCommandSummary(command, 0., standing_ready ? "Q-STAND-HOLD"
                                                       : "Q-CURRENT-HOLD");
        next_print = now + std::chrono::seconds(1);
      }
    } else if (mode == Mode::kMitStandArming ||
               (mode == Mode::kKey3Locomotion &&
                key3_phase == Key3Phase::kMitForwardArming)) {
      state_adapter->FillCheetahData(snapshot, spi_data, vector_nav_data);
      FillCheaterFromLowState(snapshot, cheater_state);
      ApplyControllerCommand(ControllerPhase::kFsmWarmup, 4.0, 0., 0., 0.,
                             robot_params, user_params, gamepad_command);
      runner->run();

      const bool forward_arm = mode == Mode::kKey3Locomotion;
      auto raw_command = command_adapter->FromSpiCommand(*spi_command);
      command_adapter->ApplyTauScale(args.tau_ff_scale, &raw_command);
      MitOutputReadiness arming_readiness;
      arming_readiness.ready = !forward_arm;
      arming_readiness.reason = forward_arm ? "time_warmup" : "ok";
      arming_readiness.reject_code = BridgeRejectCode::kOk;
      const auto command = command_adapter->MakeQOnlyCommand(
          forward_arm ? key3_hold_q : legbot::StandQDds(), args.standup_kp,
          args.standup_kd);
      legbot::OutputGuardStats arming_guard;
      PopulateRejectedHoldStats(raw_command, command, &arming_guard);
      if (!command_adapter->PublishLowCmd(lowcmd_pub, command, stats,
                                          stats_t0)) {
        std::cerr << "[LEGBOT-DDS][ERROR] publish_failed during mit-fsm-arming\n";
        command_adapter->DisableBurst(lowcmd_pub, args, stats, stats_t0);
        return "publish_failed:mit-fsm-arming";
      }

      const double arm_elapsed = legbot::SecondsSince(mit_arm_start, now);
      if (now >= next_print) {
        std::cout << "[LEGBOT-DDS][MIT-FSM-ARMING] t=" << std::fixed
                  << std::setprecision(2) << arm_elapsed
                  << " publish=Q-STAND-HOLD control_mode=STAND_UP"
                  << " requested_gait=4 cmpc_gait="
                  << user_params->cmpc_gait
                  << " ready=" << (arming_readiness.ready ? "yes" : "no")
                  << " reject="
                  << BridgeRejectCodeText(arming_readiness.reject_code)
                  << " reason=" << arming_readiness.reason
                  << " elapsed_in_phase=" << arm_elapsed << "\n";
        PrintCommandSummary(command, 0., "Q-STAND-HOLD");
        const double print_period_s =
            key3_phase == Key3Phase::kGait0Shadow ? 0.1 : 0.2;
        next_print = now + std::chrono::duration_cast<legbot::Clock::duration>(
                                  std::chrono::duration<double>(print_period_s));
      }
      if (forward_arm) {
        csv_logger.MaybeWrite(now, "KEY3_ARMING", 4.0, user_params,
                              robot_params, snapshot, *state_adapter,
                              arming_guard, runner, controller,
                              arming_readiness.ready,
                              arming_readiness.reject_code,
                              Key3PhaseText(key3_phase), 0.0, 0.0, 0.0);
      }
      const bool force_shadow =
          forward_arm && arm_elapsed >= kKey3ArmingForceShadowSeconds;
      if (force_shadow && !key3_timeout_logged) {
        key3_timeout_logged = true;
        std::cerr << "[LEGBOT-DDS][KEY3-PHASE][WARN] arming time exceeded, forcing LOCOMOTION_GAIT4_SHADOW because STAND_UP readiness is not required"
                  << " elapsed_in_phase=" << std::fixed
                  << std::setprecision(3) << arm_elapsed << "\n";
      }
      if ((!forward_arm && arm_elapsed >= args.fsm_warmup_seconds &&
           arming_readiness.ready) ||
          (forward_arm &&
           (arm_elapsed >= args.fsm_warmup_seconds || force_shadow))) {
        mit_output_wait_start = now;
        previous_q_des = forward_arm ? key3_hold_q : legbot::StandQDds();
        previous_qd_des.fill(0.);
        has_previous_q_des = true;
        if (forward_arm) {
          robot_params->control_mode = kLocomotionControlMode;
          user_params->cmpc_gait = 4.0;
          set_key3_phase(Key3Phase::kLocomotionGait4Shadow,
                         force_shadow ? "arming_time_exceeded_force_shadow"
                                      : "time_warmup_complete",
                         false, BridgeRejectCode::kOk);
        } else {
          mode = Mode::kMitStandOutputWait;
          robot_params->control_mode = kBalanceStandControlMode;
        }
        next_print = now;
        std::cout << "[LEGBOT-DDS][MIT-FSM-ARMING] done; switch to "
                  << (forward_arm ? Key3PhaseText(key3_phase)
                                  : "MIT-BALANCE-STAND-OUTPUT-WAIT")
                  << "\n";
      }
    } else if (mode == Mode::kMitStandOutputWait) {
      state_adapter->FillCheetahData(snapshot, spi_data, vector_nav_data);
      FillCheaterFromLowState(snapshot, cheater_state);

      const double requested_gait = 4.0;
      const double requested_vx = 0.0;
      ApplyControllerCommand(ControllerPhase::kBalanceStand, 4.0, 0., 0., 0.,
                             robot_params, user_params, gamepad_command);
      runner->run();

      auto raw_command = command_adapter->FromSpiCommand(*spi_command);
      command_adapter->ApplyTauScale(args.tau_ff_scale, &raw_command);
      const auto readiness = InspectMitOutputReadiness(raw_command);

      const auto command = command_adapter->MakeQOnlyCommand(
          legbot::StandQDds(), args.standup_kp, args.standup_kd);
      if (!command_adapter->PublishLowCmd(lowcmd_pub, command, stats,
                                          stats_t0)) {
        std::cerr << "[LEGBOT-DDS][ERROR] publish_failed during mit-output-wait\n";
        command_adapter->DisableBurst(lowcmd_pub, args, stats, stats_t0);
        return "publish_failed:mit-output-wait";
      }

      const double wait_elapsed =
          legbot::SecondsSince(mit_output_wait_start, now);
      if (now >= next_print) {
        PrintMitOutputWaitLog("MIT-BALANCE-STAND-OUTPUT-WAIT",
                              wait_elapsed, requested_vx, requested_gait,
                              snapshot, *state_adapter, raw_command,
                              readiness, controller, runner, robot_params,
                              user_params, gamepad_command);
        next_print = now + std::chrono::duration_cast<legbot::Clock::duration>(
                                  std::chrono::duration<double>(0.2));
      }
      if (readiness.ready) {
        mode = Mode::kMitStandDirect;
        previous_q_des = legbot::StandQDds();
        previous_qd_des.fill(0.);
        has_previous_q_des = true;
        next_print = now;
        std::cout << "[LEGBOT-DDS][MIT-OUTPUT-WAIT] valid SpiCommand; switch to "
                  << "MIT-BALANCE-STAND-DIRECT\n";
      } else if (wait_elapsed >= args.mit_output_wait_seconds) {
        std::cerr << "[LEGBOT-DDS][SOFT-FAULT] MIT SpiCommand not ready after "
                  << std::fixed << std::setprecision(2) << wait_elapsed
                  << "s reason=" << readiness.reason
                  << "; stop MIT output, back to Q-STAND-HOLD. Press 4 to exit.\n";
        mode = Mode::kQHold;
        previous_q_des = legbot::StandQDds();
        previous_qd_des.fill(0.);
        has_previous_q_des = true;
        next_print = now;
      }
    } else if (mode == Mode::kKey3Locomotion) {
      state_adapter->FillCheetahData(snapshot, spi_data, vector_nav_data);
      FillCheaterFromLowState(snapshot, cheater_state);

      const double phase_elapsed = legbot::SecondsSince(key3_phase_start, now);
      const bool gait0_phase = key3_phase == Key3Phase::kGait0Shadow ||
                               key3_phase == Key3Phase::kGait0Blend ||
                               key3_phase == Key3Phase::kGait0Direct;
      const bool shadow_phase =
          key3_phase == Key3Phase::kLocomotionGait4Shadow ||
          key3_phase == Key3Phase::kGait0Shadow;
      const double requested_gait = gait0_phase ? args.test_gait : 4.0;

      ApplyControllerCommand(ControllerPhase::kLocomotionTest,
                             requested_gait, 0., 0., 0., robot_params,
                             user_params, gamepad_command);
      runner->run();

      auto raw_command = command_adapter->FromSpiCommand(*spi_command);
      command_adapter->ApplyTauScale(args.tau_ff_scale, &raw_command);
      legbot::DdsCommandArrays command_for_guard = raw_command;
      const bool gait0_compute_only =
          key3_phase == Key3Phase::kGait0Shadow;
      if (gait0_compute_only && !gait0_entry_reference_settled &&
          phase_elapsed >= kGait0EntryReferenceSettleSeconds) {
        gait0_entry_q_fb = snapshot.dds_q;
        gait0_entry_q_hold = key3_last_safe_command.q;
        gait0_previous_entry_violation =
            std::numeric_limits<double>::quiet_NaN();
        gait0_entry_violation_increase_count = 0;
        gait0_raw_stable_count = 0;
        gait0_entry_reference_settled = true;
        std::cout << "[GAIT0-RELEASE-GATE-DIAG]"
                  << " reason=entry_reference_refreshed_after_q_hold_settle"
                  << " compute_only_elapsed=" << phase_elapsed
                  << " compute_only_required="
                  << kGait0ShadowMinComputeOnlySeconds
                  << " settle_elapsed=" << phase_elapsed
                  << " settle_required="
                  << kGait0EntryReferenceSettleSeconds
                  << " settle_s=" << kGait0EntryReferenceSettleSeconds
                  << " elapsed_in_phase=" << phase_elapsed
                  << " stable_count=" << gait0_raw_stable_count
                  << " stable_count_required="
                  << kGait0RawStableCountRequired
                  << " swing_phase_boundary_margin="
                  << kGait0SwingPhaseBoundaryMargin
                  << " raw_vs_hold_limit_hip="
                  << kGait0RawVsHoldTakeoverHip
                  << " raw_vs_hold_limit_thigh="
                  << kGait0RawVsHoldTakeoverThigh
                  << " raw_vs_hold_limit_calf="
                  << kGait0RawVsHoldTakeoverCalf
                  << " entry_reference_q_feedback="
                  << legbot::ArrayText(gait0_entry_q_fb, 5)
                  << " entry_reference_q_hold="
                  << legbot::ArrayText(gait0_entry_q_hold, 5)
                  << " last_safe_q_des="
                  << legbot::ArrayText(key3_last_safe_command.q, 5)
                  << " release_allowed=false"
                  << " release_block_reason=entry_reference_settle"
                  << " publish=KEY3_LAST_SAFE_COMMAND"
                  << "\n";
      }
      const auto gait0_entry_violation =
          MaxAbsQStep(raw_command, gait0_entry_q_fb);
      const auto gait0_raw_vs_hold_step =
          MaxAbsQStep(raw_command, gait0_entry_q_hold);
      const double gait0_raw_vs_hold_limit =
          gait0_raw_vs_hold_step.joint >= 0
              ? Gait0RawVsHoldTakeoverLimitForDdsJoint(
                    gait0_raw_vs_hold_step.joint)
              : std::numeric_limits<double>::infinity();
      JointDiffDiagnostic gait0_raw_q_step;
      if (gait0_has_previous_raw_q) {
        gait0_raw_q_step = MaxAbsQStep(raw_command, gait0_previous_raw_q);
      }
      const double gait0_max_abs_raw_qd = MaxAbsRawQd(raw_command);
      const double gait0_max_abs_raw_tau = MaxAbsRawTau(raw_command);
      const auto* cmpc_debug_for_diag =
          controller ? controller->debugConvexMPC() : nullptr;
      const Gait0MpcTrace gait0_current_mpc_trace =
          CaptureGait0MpcTrace(cmpc_debug_for_diag);
      const double cmpc_foot_jump_diagnostic =
          DebugMaxCmpcFootPDesJump(cmpc_debug_for_diag);
      const double leg_command_foot_jump_diagnostic =
          DebugMaxFootPDesJump(runner);
      const double foot_jump_diagnostic =
          (gait0_phase && std::isfinite(cmpc_foot_jump_diagnostic))
              ? cmpc_foot_jump_diagnostic
              : leg_command_foot_jump_diagnostic;
      const double cmpc_sum_fz_diagnostic =
          DebugCmpcSumFrDesZ(cmpc_debug_for_diag);
      const double leg_command_sum_fz_diagnostic = DebugWbcSumFz(runner);
      const double wbc_sum_fz_diagnostic =
          (gait0_phase && std::isfinite(cmpc_sum_fz_diagnostic))
              ? cmpc_sum_fz_diagnostic
              : leg_command_sum_fz_diagnostic;
      const bool wbc_force_hard_gate =
          key3_phase == Key3Phase::kGait0Direct;
      const double wbc_force_hard_min =
          wbc_force_hard_gate ? kWbcForceHardMinFzGait0Direct
                              : kWbcForceWarnMinFz;
      const bool foot_jump_hard_gate =
          key3_phase == Key3Phase::kGait0Shadow;
      const double foot_target_jump_hard_limit =
          foot_jump_hard_gate ? kFootTargetJumpHardLimitGait0
                              : kFootTargetJumpHardLimitDefault;
      if (gait0_compute_only && gait0_current_mpc_trace.valid) {
        if (!gait0_window_has_mpc_table) {
          gait0_window_has_mpc_table = true;
          gait0_window_last_mpc_table = gait0_current_mpc_trace.mpc_table_now;
        } else if (gait0_current_mpc_trace.mpc_table_now !=
                   gait0_window_last_mpc_table) {
          ++gait0_window_mpc_table_transitions;
          gait0_window_last_mpc_table =
              gait0_current_mpc_trace.mpc_table_now;
        }
      }
      const bool gait0_raw_q_step_too_large =
          gait0_compute_only && gait0_has_previous_raw_q &&
          gait0_raw_q_step.joint >= 0 &&
          (!std::isfinite(gait0_raw_q_step.value) ||
           gait0_raw_q_step.value >= kGait0RawStableQStepLimit);
      if (gait0_compute_only && gait0_has_previous_raw_q &&
          gait0_raw_q_step.joint >= 0 &&
          std::isfinite(gait0_raw_q_step.value)) {
        gait0_window_max_raw_q_step =
            std::max(gait0_window_max_raw_q_step, gait0_raw_q_step.value);
      }
      if (gait0_raw_q_step_too_large) {
        PrintRawQStepTooLargeEvent(
            gait0_raw_q_step, raw_command, gait0_previous_raw_q, snapshot,
            gait0_previous_mpc_trace, gait0_current_mpc_trace,
            cmpc_debug_for_diag, runner);
        gait0_window_start = now;
        gait0_window_mpc_table_transitions = 0;
        gait0_window_has_mpc_table = gait0_current_mpc_trace.valid;
        gait0_window_last_mpc_table =
            gait0_current_mpc_trace.valid
                ? gait0_current_mpc_trace.mpc_table_now
                : std::array<double, 4>{};
        gait0_window_max_raw_q_step = 0.;
        gait0_window_bad_raw_q_step = false;
        gait0_raw_stable_count = 0;
      }
      std::string gait0_raw_release_block_reason = "released";
      MitOutputReadiness bridge_ready;
      if (gait0_compute_only) {
        bridge_ready = InspectGait0ShadowReleaseGate(
            raw_command, gait0_entry_q_fb, snapshot, *state_adapter, args,
            gait0_raw_q_step, gait0_has_previous_raw_q,
            gait0_max_abs_raw_qd, gait0_max_abs_raw_tau,
            foot_jump_diagnostic);
        const bool gait0_raw_step_stable =
            gait0_has_previous_raw_q && gait0_raw_q_step.joint >= 0 &&
            std::isfinite(gait0_raw_q_step.value) &&
            gait0_raw_q_step.value < kGait0RawStableQStepLimit;
        const bool gait0_raw_vs_hold_blendable =
            gait0_raw_vs_hold_step.joint < 0 ||
            (std::isfinite(gait0_raw_vs_hold_step.value) &&
             gait0_raw_vs_hold_step.value <= gait0_raw_vs_hold_limit);
        if (bridge_ready.ready && !gait0_entry_reference_settled) {
          std::ostringstream oss;
          oss << "gait0_entry_reference_settle elapsed=" << phase_elapsed
              << " required=" << kGait0EntryReferenceSettleSeconds;
          bridge_ready.ready = false;
          bridge_ready.reject_code = BridgeRejectCode::kPreTrotTimeout;
          bridge_ready.reason = oss.str();
        }
        const double gait0_entry_violation_previous =
            gait0_previous_entry_violation;
        bool gait0_entry_violation_increasing = false;
        if (std::isfinite(gait0_entry_violation.value)) {
          if (std::isfinite(gait0_entry_violation_previous) &&
              gait0_entry_violation.value >
                  gait0_entry_violation_previous + 0.005) {
            ++gait0_entry_violation_increase_count;
            gait0_entry_violation_increasing = true;
          } else {
            gait0_entry_violation_increase_count = 0;
          }
          gait0_previous_entry_violation = gait0_entry_violation.value;
        }
        if (bridge_ready.ready && args.gait0_never_release) {
          bridge_ready.ready = false;
          bridge_ready.reject_code = BridgeRejectCode::kPreTrotTimeout;
          bridge_ready.reason = "gait0_never_release";
        }
        if (bridge_ready.ready &&
            phase_elapsed < kGait0ShadowMinComputeOnlySeconds) {
          std::ostringstream oss;
          oss << "gait0_compute_only_min_time elapsed=" << phase_elapsed
              << " required=" << kGait0ShadowMinComputeOnlySeconds;
          bridge_ready.ready = false;
          bridge_ready.reject_code = BridgeRejectCode::kPreTrotTimeout;
          bridge_ready.reason = oss.str();
        }
        if (bridge_ready.ready &&
            gait0_window_mpc_table_transitions <
                kGait0ShadowMinMpcTableTransitions) {
          std::ostringstream oss;
          oss << "gait0_wait_full_trot_cycle transitions="
              << gait0_window_mpc_table_transitions
              << " required=" << kGait0ShadowMinMpcTableTransitions;
          bridge_ready.ready = false;
          bridge_ready.reject_code = BridgeRejectCode::kPreTrotTimeout;
          bridge_ready.reason = oss.str();
        }
        if (bridge_ready.ready &&
            (gait0_raw_q_step_too_large || gait0_window_bad_raw_q_step ||
             gait0_window_max_raw_q_step >= kGait0RawStableQStepLimit)) {
          std::ostringstream oss;
          oss << "gait0_window_raw_q_step_not_clean max_raw_q_step="
              << gait0_window_max_raw_q_step
              << " current_raw_q_step=" << gait0_raw_q_step.value
              << " limit=" << kGait0RawStableQStepLimit;
          bridge_ready.ready = false;
          bridge_ready.reject_code = BridgeRejectCode::kQDesJump;
          bridge_ready.reason = oss.str();
        }
        if (bridge_ready.ready &&
            (!std::isfinite(gait0_max_abs_raw_tau) ||
             gait0_max_abs_raw_tau >= kGait0ShadowMaxAbsTau)) {
          std::ostringstream oss;
          oss << "raw_tau_too_large max_abs_raw_tau="
              << gait0_max_abs_raw_tau
              << " limit=" << kGait0ShadowMaxAbsTau;
          bridge_ready.ready = false;
          bridge_ready.reject_code = BridgeRejectCode::kWbcForceTooLow;
          bridge_ready.reason = oss.str();
        }
        if (bridge_ready.ready &&
            gait0_entry_violation_increase_count >= 5) {
          std::ostringstream oss;
          oss << "entry_violation_increasing count="
              << gait0_entry_violation_increase_count
              << " max_entry_violation=" << gait0_entry_violation.value;
          if (gait0_entry_violation.joint >= 0) {
            oss << " joint=" << legbot::JointNameDds(gait0_entry_violation.joint);
          }
          const bool entry_increase_hard_block =
              !gait0_raw_step_stable || !gait0_raw_vs_hold_blendable;
          std::cout << "[GAIT0-RELEASE-GATE-DIAG]"
                    << " reason="
                    << (entry_increase_hard_block
                            ? "entry_violation_increasing_hard_block"
                            : "entry_violation_increasing_soft_allow")
                    << " compute_only_elapsed=" << phase_elapsed
                    << " compute_only_required="
                    << kGait0ShadowMinComputeOnlySeconds
                    << " settle_elapsed=" << phase_elapsed
                    << " settle_required="
                    << kGait0EntryReferenceSettleSeconds
                    << " count=" << gait0_entry_violation_increase_count
                    << " max_entry_violation="
                    << gait0_entry_violation.value
                    << " max_entry_violation_joint="
                    << (gait0_entry_violation.joint >= 0
                            ? legbot::JointNameDds(gait0_entry_violation.joint)
                            : "unknown")
                    << " previous_max_entry_violation="
                    << gait0_entry_violation_previous
                    << " current_max_entry_violation="
                    << gait0_entry_violation.value
                    << " increasing="
                    << (gait0_entry_violation_increasing ? "true" : "false")
                    << " q_feedback=" << legbot::ArrayText(snapshot.dds_q, 5)
                    << " entry_reference_q_feedback="
                    << legbot::ArrayText(gait0_entry_q_fb, 5)
                    << " last_safe_q_des="
                    << legbot::ArrayText(key3_last_safe_command.q, 5)
                    << " entry_reference_q_hold="
                    << legbot::ArrayText(gait0_entry_q_hold, 5)
                    << " gait0_raw_q_des="
                    << legbot::ArrayText(raw_command.q, 5)
                    << " raw_q_step_stable="
                    << (gait0_raw_step_stable ? "true" : "false")
                    << " raw_q_step=" << gait0_raw_q_step.value
                    << " raw_q_step_limit=" << kGait0RawStableQStepLimit
                    << " raw_vs_hold_joint="
                    << (gait0_raw_vs_hold_step.joint >= 0
                            ? legbot::JointNameDds(gait0_raw_vs_hold_step.joint)
                            : "unknown")
                    << " max_raw_vs_hold=" << gait0_raw_vs_hold_step.value
                    << " max_raw_vs_hold_joint="
                    << (gait0_raw_vs_hold_step.joint >= 0
                            ? legbot::JointNameDds(gait0_raw_vs_hold_step.joint)
                            : "unknown")
                    << " raw_vs_hold_step=" << gait0_raw_vs_hold_step.value
                    << " raw_vs_hold_limit=" << gait0_raw_vs_hold_limit
                    << " raw_vs_hold_limit_hip="
                    << kGait0RawVsHoldTakeoverHip
                    << " raw_vs_hold_limit_thigh="
                    << kGait0RawVsHoldTakeoverThigh
                    << " raw_vs_hold_limit_calf="
                    << kGait0RawVsHoldTakeoverCalf
                    << " raw_vs_hold_blendable="
                    << (gait0_raw_vs_hold_blendable ? "true" : "false")
                    << " stable_count="
                    << gait0_raw_stable_count
                    << " stable_count_required="
                    << kGait0RawStableCountRequired
                    << " gait0_window_max_raw_q_step="
                    << gait0_window_max_raw_q_step
                    << " swing_phase_boundary_margin="
                    << kGait0SwingPhaseBoundaryMargin
                    << " release_allowed="
                    << (entry_increase_hard_block ? "false" : "true")
                    << " release_block_reason="
                    << (entry_increase_hard_block
                            ? "entry_violation_increasing"
                            : "none")
                    << " publish=KEY3_LAST_SAFE_COMMAND"
                    << "\n";
          if (entry_increase_hard_block) {
            bridge_ready.ready = false;
            bridge_ready.reject_code = BridgeRejectCode::kQDesJump;
            bridge_ready.reason = oss.str();
          } else {
            gait0_entry_violation_increase_count = 0;
          }
        }
        bool gait0_soft_phase_boundary = false;
        if (bridge_ready.ready) {
          const auto* cmpc = controller ? controller->debugConvexMPC()
                                        : nullptr;
          if (!cmpc || !cmpc->valid) {
            bridge_ready.ready = false;
            bridge_ready.reject_code = BridgeRejectCode::kContactStateNotReady;
            bridge_ready.reason = "cmpc_debug_unavailable";
          } else {
            std::array<double, 4> is_swing_leg{};
            std::array<double, 4> boundary_checked{};
            std::array<double, 4> boundary_blocked{};
            for (int leg = 0; leg < 4; ++leg) {
              const double swing_phase =
                  static_cast<double>(cmpc->swingStates[leg]);
              const bool leg_is_swing = cmpc->mpcTableNow[leg] == 0;
              is_swing_leg[leg] = leg_is_swing ? 1.0 : 0.0;
              boundary_checked[leg] = leg_is_swing ? 1.0 : 0.0;
              const bool leg_boundary_blocked =
                  leg_is_swing &&
                  (!std::isfinite(swing_phase) ||
                  swing_phase <= kGait0SwingPhaseBoundaryMargin ||
                   swing_phase >= 1.0 - kGait0SwingPhaseBoundaryMargin);
              boundary_blocked[leg] = leg_boundary_blocked ? 1.0 : 0.0;
              if (leg_boundary_blocked) {
                std::ostringstream oss;
                oss << "swing_phase_boundary leg=" << leg
                    << " phase=" << swing_phase
                    << " margin=" << kGait0SwingPhaseBoundaryMargin
                    << " is_swing_leg=true"
                    << " boundary_checked=true"
                    << " boundary_blocked=true"
                    << " swing_phase="
                    << legbot::ArrayText(Vec4TextArray(cmpc->swingStates), 3)
                    << " cmpc_mpc_table_now="
                    << legbot::ArrayText(Vec4iTextArray(cmpc->mpcTableNow), 0)
                    << " firstSwing="
                    << legbot::ArrayText(Vec4iTextArray(cmpc->firstSwing), 0)
                    << " is_swing_leg="
                    << legbot::ArrayText(is_swing_leg, 0)
                    << " boundary_checked="
                    << legbot::ArrayText(boundary_checked, 0)
                    << " boundary_blocked="
                    << legbot::ArrayText(boundary_blocked, 0);
                std::cout << "[GAIT0-RELEASE-GATE-DIAG]"
                          << " reason=swing_phase_boundary"
                          << " compute_only_elapsed=" << phase_elapsed
                          << " compute_only_required="
                          << kGait0ShadowMinComputeOnlySeconds
                          << " settle_elapsed=" << phase_elapsed
                          << " settle_required="
                          << kGait0EntryReferenceSettleSeconds
                          << " leg=" << leg
                          << " swing_phase="
                          << legbot::ArrayText(Vec4TextArray(cmpc->swingStates),
                                             3)
                          << " cmpc_mpc_table_now="
                          << legbot::ArrayText(Vec4iTextArray(cmpc->mpcTableNow),
                                             0)
                          << " firstSwing="
                          << legbot::ArrayText(Vec4iTextArray(cmpc->firstSwing),
                                             0)
                          << " is_swing_leg="
                          << legbot::ArrayText(is_swing_leg, 0)
                          << " boundary_checked="
                          << legbot::ArrayText(boundary_checked, 0)
                          << " boundary_blocked="
                          << legbot::ArrayText(boundary_blocked, 0)
                          << " stable_count="
                          << gait0_raw_stable_count
                          << " stable_count_required="
                          << kGait0RawStableCountRequired
                          << " gait0_window_max_raw_q_step="
                          << gait0_window_max_raw_q_step
                          << " max_entry_violation="
                          << gait0_entry_violation.value
                          << " max_entry_violation_joint="
                          << (gait0_entry_violation.joint >= 0
                                  ? legbot::JointNameDds(
                                        gait0_entry_violation.joint)
                                  : "unknown")
                          << " swing_phase_boundary_margin="
                          << kGait0SwingPhaseBoundaryMargin
                          << " raw_vs_hold_limit_hip="
                          << kGait0RawVsHoldTakeoverHip
                          << " raw_vs_hold_limit_thigh="
                          << kGait0RawVsHoldTakeoverThigh
                          << " raw_vs_hold_limit_calf="
                          << kGait0RawVsHoldTakeoverCalf
                          << " max_raw_vs_hold="
                          << gait0_raw_vs_hold_step.value
                          << " max_raw_vs_hold_joint="
                          << (gait0_raw_vs_hold_step.joint >= 0
                                  ? legbot::JointNameDds(
                                        gait0_raw_vs_hold_step.joint)
                                  : "unknown")
                          << " release_allowed=false"
                          << " release_block_reason=swing_phase_boundary"
                          << " publish=KEY3_LAST_SAFE_COMMAND"
                          << "\n";
                bridge_ready.ready = false;
                bridge_ready.reject_code = BridgeRejectCode::kSwingPhaseNotReady;
                bridge_ready.reason = oss.str();
                gait0_soft_phase_boundary = true;
                break;
              }
            }
          }
        }
        if (bridge_ready.ready) {
          ++gait0_raw_stable_count;
        } else if (!gait0_soft_phase_boundary) {
          gait0_raw_stable_count = 0;
        }

        if (bridge_ready.ready &&
            gait0_raw_stable_count >= kGait0RawStableCountRequired) {
          gait0_raw_released = true;
          gait0_raw_release_block_reason = "released";
        } else {
          gait0_raw_released = false;
          gait0_raw_release_block_reason =
              bridge_ready.ready ? "waiting_for_raw_stable_count"
                                 : bridge_ready.reason;
          bridge_ready.ready = false;
          bridge_ready.reject_code = BridgeRejectCode::kPreTrotTimeout;
          bridge_ready.reason =
              std::string("gait0_shadow_release_gate:") +
              gait0_raw_release_block_reason;
        }
      } else if (gait0_phase) {
        gait0_raw_released = true;
        bridge_ready.ready = true;
        bridge_ready.reason = "ok";
        bridge_ready.reject_code = BridgeRejectCode::kOk;
      } else {
        constexpr bool kGait4AllStanceCheck = true;
        constexpr bool kGait0ShadowEntryCheck = false;
        bridge_ready = InspectKey3LocomotionReadiness(
            command_for_guard, snapshot, *state_adapter, args, runner,
            controller, kGait4AllStanceCheck, kGait0ShadowEntryCheck,
            false /* wbc_force_hard_gate */, kKey3RawWarmupSeconds,
            "locomotion_raw_warmup", phase_elapsed);
      }
      const bool gait4_direct_raw_warmup =
          key3_phase == Key3Phase::kLocomotionGait4Direct &&
          bridge_ready.reject_code == BridgeRejectCode::kPreTrotTimeout &&
          bridge_ready.reason.find("locomotion_raw_warmup") !=
              std::string::npos;
      const std::string bridge_log_reason =
          gait4_direct_raw_warmup ? "gait4_direct_raw_warmup_hold"
                                  : bridge_ready.reason;
      std::string publish_tag = "RAW_GUARDED";
      gait0_previous_raw_q = raw_command.q;
      gait0_has_previous_raw_q = true;
      gait0_previous_mpc_trace = gait0_current_mpc_trace;
      if (key3_phase == Key3Phase::kGait0Shadow &&
          !key3_gait0_entry_logged) {
        key3_gait0_entry_logged = true;
        PrintGait0EntryDiagnostics(raw_command, snapshot, runner, controller);
      }
      legbot::OutputGuardStats guard;
      double alpha_q = 1.0;
      double alpha_qd = 1.0;
      double alpha_tau = 1.0;
      double gait0_blend_entry_limit = std::numeric_limits<double>::infinity();
      bool gait0_blend_entry_violation = false;
      const bool gait0_blend_phase = key3_phase == Key3Phase::kGait0Blend;
      if (gait0_entry_violation.joint >= 0) {
        gait0_blend_entry_limit = ReadyMaxQErrorForDdsJoint(
            gait0_entry_violation.joint, true /* gait0_shadow_entry_check */);
        gait0_blend_entry_violation =
            gait0_entry_violation.value > gait0_blend_entry_limit;
      }
      const bool gait0_blend_entry_soft_hold =
          gait0_blend_phase && gait0_blend_entry_violation &&
          phase_elapsed <= kGait0BlendEntryEnvelopeSoftHoldSeconds;
      const char* gait0_blend_entry_envelope_mode =
          !gait0_blend_phase
              ? "hard_block"
              : (gait0_blend_entry_violation
                     ? (gait0_blend_entry_soft_hold ? "soft_hold"
                                                    : "disabled_after_blend")
                     : "warn_only");

      auto publish_hold_with_raw_stats =
          [&](const legbot::DdsCommandArrays& hold_command) -> bool {
        PopulateRejectedHoldStats(command_for_guard, hold_command, &guard);
        return command_adapter->PublishLowCmd(lowcmd_pub, guard.pub, stats,
                                              stats_t0);
      };

      auto apply_guarded_raw = [&]() -> bool {
        if (args.real_output_raw) {
          return PopulateRawCommandStats(command_for_guard, &guard);
        }
        return legbot::ApplyOutputGuard(
            command_for_guard, args,
            has_previous_q_des ? &previous_q_des : nullptr,
            has_previous_q_des ? &previous_qd_des : nullptr, &guard);
      };

      auto apply_gait0_blend_softening = [&]() {
        if (key3_phase != Key3Phase::kGait0Blend) return;

        const double target_alpha_q =
            legbot::Clamp(phase_elapsed / kKey3BlendQSeconds, 0.0, 1.0);
        if (gait0_blend_entry_soft_hold) {
          gait0_blend_alpha_q = std::min(gait0_blend_alpha_q, target_alpha_q);
        } else if (target_alpha_q >= gait0_blend_alpha_q) {
          gait0_blend_alpha_q =
              std::min(target_alpha_q,
                       gait0_blend_alpha_q + kGait0BlendMaxAlphaStep);
        } else {
          gait0_blend_alpha_q = target_alpha_q;
        }
        alpha_q = gait0_blend_alpha_q;
        alpha_qd = kGait0BlendQdAlpha;
        alpha_tau = kGait0BlendTauAlphaScale * alpha_q;

        for (std::size_t i = 0; i < legbot::kNumJoints; ++i) {
          const int dds_joint = static_cast<int>(i);
          const int model_joint = legbot::DdsToModelJoint(dds_joint);
          const int model_leg =
              model_joint / static_cast<int>(legbot::kNumJointsPerLeg);
          const bool stance_leg =
              cmpc_debug_for_diag && cmpc_debug_for_diag->valid &&
              model_leg >= 0 && model_leg < 4 &&
              cmpc_debug_for_diag->mpcTableNow[model_leg] == 1;
          const double joint_alpha_q =
              legbot::Clamp(alpha_q * Gait0BlendAlphaScaleForDdsJoint(dds_joint),
                          0.0, 1.0);
          const double joint_q_step_limit =
              Gait0BlendMaxQStepForDdsJoint(dds_joint);
          const double joint_alpha_tau =
              legbot::Clamp(alpha_q *
                              Gait0BlendTauAlphaScaleForDdsJoint(dds_joint),
                          0.0, 1.0);

          const double q_start = gait0_blend_start_q[i];
          const double q_target =
              q_start + joint_alpha_q * (raw_command.q[i] - q_start);
          const double q_prev =
              has_previous_q_des ? previous_q_des[i] : key3_last_safe_command.q[i];
          command_for_guard.q[i] =
              q_prev + legbot::Clamp(q_target - q_prev,
                                   -joint_q_step_limit,
                                   joint_q_step_limit);

          command_for_guard.dq[i] =
              kGait0BlendQdAlpha *
              (raw_command.dq[i] - gait0_blend_start_dq[i]);
          command_for_guard.tau[i] =
              gait0_blend_start_tau[i] +
              joint_alpha_tau *
                  (raw_command.tau[i] - gait0_blend_start_tau[i]);
          if (stance_leg) {
            command_for_guard.kp[i] =
                std::max(command_for_guard.kp[i],
                         Gait0BlendStanceKpMinForDdsJoint(dds_joint));
            command_for_guard.kd[i] =
                std::max(command_for_guard.kd[i],
                         Gait0BlendStanceKdMinForDdsJoint(dds_joint));
          }
        }
        publish_tag = "GAIT0_BLEND_SOFT";
      };

      const auto q_hold_command = command_adapter->MakeQOnlyCommand(
          key3_hold_q, args.standup_kp, args.standup_kd);
      const auto q_stand_hold_command = command_adapter->MakeQOnlyCommand(
          legbot::StandQDds(), args.standup_kp, args.standup_kd);
      auto abort_gait0_to_qstand =
          [&](const std::string& reason, const std::string& detail = "") -> bool {
        std::cout << "[LEGBOT-DDS][GAIT0-BLEND-MONITOR] abort reason="
                  << reason
                  << " phase=" << Key3PhaseText(key3_phase)
                  << " requested_gait=4 publish=Q-STAND-HOLD";
        if (!detail.empty()) std::cout << " " << detail;
        std::cout << "\n";
        legbot::OutputGuardStats hold_guard;
        PopulateRejectedHoldStats(command_for_guard, q_stand_hold_command,
                                  &hold_guard);
        const bool ok = command_adapter->PublishLowCmd(
            lowcmd_pub, hold_guard.pub, stats, stats_t0);
        guard = hold_guard;
        mode = Mode::kQHold;
        key3_phase = Key3Phase::kNone;
        user_params->cmpc_gait = 4.0;
        robot_params->control_mode = kStandUpControlMode;
        gait0_raw_released = false;
        gait0_monitor_qdes_clamped_count = 0;
        gait0_monitor_entry_violation_count = 0;
        previous_q_des = legbot::StandQDds();
        previous_qd_des.fill(0.);
        has_previous_q_des = true;
        publish_tag = "Q-STAND-HOLD";
        return ok;
      };
      bool publish_ok = true;

      if (shadow_phase) {
        publish_tag = key3_phase == Key3Phase::kGait0Shadow
                          ? "KEY3_LAST_SAFE_COMMAND"
                          : "Q-STAND-HOLD";
        const bool ready_now = bridge_ready.ready;
        if (ready_now && !key3_has_stable_start) {
          key3_has_stable_start = true;
          key3_stable_start = now;
        } else if (!ready_now) {
          key3_has_stable_start = false;
        }
        const double stable_s =
            key3_has_stable_start ? legbot::SecondsSince(key3_stable_start, now)
                                  : 0.0;

        const auto& shadow_hold =
            key3_phase == Key3Phase::kGait0Shadow ? key3_last_safe_command
                                                   : q_hold_command;
        publish_ok = publish_hold_with_raw_stats(shadow_hold);
        alpha_q = 0.0;
        alpha_qd = 0.0;
        alpha_tau = 0.0;

        if (key3_phase == Key3Phase::kLocomotionGait4Shadow &&
            std::isfinite(foot_jump_diagnostic) &&
            foot_jump_diagnostic > kFootTargetJumpWarnLimit &&
            !key3_gait4_foot_jump_diag_logged) {
          key3_gait4_foot_jump_diag_logged = true;
          std::cout << "[LEGBOT-DDS][KEY3-PHASE] gait4 foot_jump diagnostic only:"
                    << " foot_jump=" << std::fixed << std::setprecision(6)
                    << foot_jump_diagnostic
                    << " foot_jump_warn_limit=" << kFootTargetJumpWarnLimit
                    << " foot_jump_hard_limit="
                    << foot_target_jump_hard_limit
                    << ", not rejecting in gait4 standing\n";
        }

        if (!ready_now && bridge_ready.reason != last_direct_reject_reason) {
          last_direct_reject_reason = bridge_ready.reason;
          std::cout << "[LEGBOT-DDS][KEY3-PHASE] phase="
                    << Key3PhaseText(key3_phase)
                    << " control_mode=" << robot_params->control_mode
                    << " requested_gait=" << requested_gait
                    << " cmpc_gait=" << user_params->cmpc_gait
                    << " ready=no reject="
                    << BridgeRejectCodeText(bridge_ready.reject_code)
                    << " reason=" << bridge_ready.reason
                    << " elapsed_in_phase=" << std::fixed
                    << std::setprecision(3) << phase_elapsed << "\n";
          if (key3_phase == Key3Phase::kGait0Shadow) {
            const auto* cmpc = controller ? controller->debugConvexMPC()
                                          : nullptr;
            if (cmpc && cmpc->valid) {
              std::array<double, 4> is_swing_leg{};
              std::array<double, 4> boundary_checked{};
              std::array<double, 4> boundary_blocked{};
              for (int leg = 0; leg < 4; ++leg) {
                const bool leg_is_swing = cmpc->mpcTableNow[leg] == 0;
                const double swing_phase =
                    static_cast<double>(cmpc->swingStates[leg]);
                is_swing_leg[leg] = leg_is_swing ? 1.0 : 0.0;
                boundary_checked[leg] = leg_is_swing ? 1.0 : 0.0;
                boundary_blocked[leg] =
                    (leg_is_swing &&
                     (!std::isfinite(swing_phase) ||
                      swing_phase <= kGait0SwingPhaseBoundaryMargin ||
                      swing_phase >= 1.0 - kGait0SwingPhaseBoundaryMargin))
                        ? 1.0
                        : 0.0;
              }
              std::cout << "[GAIT0-RELEASE-GATE-DIAG]"
                        << " reason=" << bridge_ready.reason
                        << " compute_only_elapsed=" << phase_elapsed
                        << " compute_only_required="
                        << kGait0ShadowMinComputeOnlySeconds
                        << " settle_elapsed=" << phase_elapsed
                        << " settle_required="
                        << kGait0EntryReferenceSettleSeconds
                        << " leg="
                        << (gait0_entry_violation.joint >= 0
                                ? gait0_entry_violation.joint / 3
                                : -1)
                        << " swing_phase="
                        << legbot::ArrayText(Vec4TextArray(cmpc->swingStates),
                                           3)
                        << " cmpc_mpc_table_now="
                        << legbot::ArrayText(Vec4iTextArray(cmpc->mpcTableNow),
                                           0)
                        << " firstSwing="
                        << legbot::ArrayText(Vec4iTextArray(cmpc->firstSwing),
                                           0)
                        << " is_swing_leg="
                        << legbot::ArrayText(is_swing_leg, 0)
                        << " boundary_checked="
                        << legbot::ArrayText(boundary_checked, 0)
                        << " boundary_blocked="
                        << legbot::ArrayText(boundary_blocked, 0)
                        << " stable_count="
                        << gait0_raw_stable_count
                        << " stable_count_required="
                        << kGait0RawStableCountRequired
                        << " gait0_window_max_raw_q_step="
                        << gait0_window_max_raw_q_step
                        << " max_entry_violation="
                        << gait0_entry_violation.value
                        << " max_entry_violation_joint="
                        << (gait0_entry_violation.joint >= 0
                                ? legbot::JointNameDds(
                                      gait0_entry_violation.joint)
                                : "unknown")
                        << " swing_phase_boundary_margin="
                        << kGait0SwingPhaseBoundaryMargin
                        << " raw_vs_hold_limit_hip="
                        << kGait0RawVsHoldTakeoverHip
                        << " raw_vs_hold_limit_thigh="
                        << kGait0RawVsHoldTakeoverThigh
                        << " raw_vs_hold_limit_calf="
                        << kGait0RawVsHoldTakeoverCalf
                        << " max_raw_vs_hold="
                        << gait0_raw_vs_hold_step.value
                        << " max_raw_vs_hold_joint="
                        << (gait0_raw_vs_hold_step.joint >= 0
                                ? legbot::JointNameDds(
                                      gait0_raw_vs_hold_step.joint)
                                : "unknown")
                        << " release_allowed=false"
                        << " release_block_reason=" << bridge_ready.reason
                        << " publish=" << publish_tag
                        << "\n";
            } else {
              std::cout << "[GAIT0-RELEASE-GATE-DIAG]"
                        << " reason=" << bridge_ready.reason
                        << " cmpc_debug=unavailable"
                        << " compute_only_elapsed=" << phase_elapsed
                        << " compute_only_required="
                        << kGait0ShadowMinComputeOnlySeconds
                        << " settle_elapsed=" << phase_elapsed
                        << " settle_required="
                        << kGait0EntryReferenceSettleSeconds
                        << " stable_count="
                        << gait0_raw_stable_count
                        << " stable_count_required="
                        << kGait0RawStableCountRequired
                        << " gait0_window_max_raw_q_step="
                        << gait0_window_max_raw_q_step
                        << " max_entry_violation="
                        << gait0_entry_violation.value
                        << " max_entry_violation_joint="
                        << (gait0_entry_violation.joint >= 0
                                ? legbot::JointNameDds(
                                      gait0_entry_violation.joint)
                                : "unknown")
                        << " swing_phase_boundary_margin="
                        << kGait0SwingPhaseBoundaryMargin
                        << " raw_vs_hold_limit_hip="
                        << kGait0RawVsHoldTakeoverHip
                        << " raw_vs_hold_limit_thigh="
                        << kGait0RawVsHoldTakeoverThigh
                        << " raw_vs_hold_limit_calf="
                        << kGait0RawVsHoldTakeoverCalf
                        << " max_raw_vs_hold="
                        << gait0_raw_vs_hold_step.value
                        << " max_raw_vs_hold_joint="
                        << (gait0_raw_vs_hold_step.joint >= 0
                                ? legbot::JointNameDds(
                                      gait0_raw_vs_hold_step.joint)
                                : "unknown")
                        << " release_allowed=false"
                        << " release_block_reason=" << bridge_ready.reason
                        << " publish=" << publish_tag
                        << "\n";
            }
          }
        }
        if (!ready_now && phase_elapsed >= kKey3ShadowMaxWaitSeconds &&
            !key3_timeout_logged) {
          key3_timeout_logged = true;
          std::cout << "[LEGBOT-DDS][KEY3-PHASE] phase="
                    << Key3PhaseText(key3_phase)
                    << " timeout_s=" << kKey3ShadowMaxWaitSeconds
                    << " hold; wait for x/4\n";
        }
        if (key3_phase == Key3Phase::kGait0Shadow &&
            gait0_raw_released) {
          key3_hold_q = key3_last_safe_command.q;
          previous_q_des = key3_last_safe_command.q;
          previous_qd_des = key3_last_safe_command.dq;
          has_previous_q_des = true;
          set_key3_phase(Key3Phase::kGait0Blend,
                         "gait0_shadow_release_gate_stable");
        } else if (ready_now && stable_s >= kKey3ShadowStableSeconds) {
          if (key3_phase == Key3Phase::kLocomotionGait4Shadow) {
            const auto* cmpc = controller ? controller->debugConvexMPC()
                                          : nullptr;
            if (cmpc && cmpc->valid) {
              std::cout << "[LEGBOT-DDS][KEY3-PHASE] gait4_shadow_ready"
                        << " stable_time=" << std::fixed
                        << std::setprecision(3) << stable_s
                        << " mpc_table_now="
                        << legbot::ArrayText(Vec4iTextArray(cmpc->mpcTableNow),
                                           0)
                        << " swing_phase="
                        << legbot::ArrayText(Vec4TextArray(cmpc->swingStates),
                                           3)
                        << " contactStates="
                        << legbot::ArrayText(Vec4TextArray(cmpc->contactStates),
                                           3)
                        << " foot_jump_diagnostic="
                        << foot_jump_diagnostic
                        << " foot_jump_hard_gate=false"
                        << "\n";
            }
            previous_q_des = key3_hold_q;
            previous_qd_des.fill(0.);
            has_previous_q_des = true;
            set_key3_phase(Key3Phase::kLocomotionGait4Direct,
                           "gait4_shadow_ready");
          }
        }
      } else {
        if (!bridge_ready.ready) {
          if (gait4_direct_raw_warmup) {
            publish_tag = "KEY3_LAST_SAFE_COMMAND";
            publish_ok = publish_hold_with_raw_stats(key3_last_safe_command);
            alpha_q = 0.0;
            alpha_qd = 0.0;
            alpha_tau = 0.0;
            if (bridge_log_reason != last_direct_reject_reason) {
              last_direct_reject_reason = bridge_log_reason;
              std::cout << "[LEGBOT-DDS][KEY3-PHASE] phase="
                        << Key3PhaseText(key3_phase)
                        << " bridge_ready=false"
                        << " bridge_reject_code="
                        << BridgeRejectCodeText(bridge_ready.reject_code)
                        << " reason=" << bridge_log_reason
                        << " publish=" << publish_tag
                        << " elapsed_in_phase=" << std::fixed
                        << std::setprecision(3) << phase_elapsed << "\n";
            }
          } else {
            if (bridge_ready.reason != last_direct_reject_reason) {
              last_direct_reject_reason = bridge_ready.reason;
              std::cout << "[LEGBOT-DDS][KEY3-PHASE] phase="
                        << Key3PhaseText(key3_phase)
                        << " reject command reason=" << bridge_ready.reason
                        << "; safe hold\n";
            }
            publish_tag = key3_phase == Key3Phase::kLocomotionGait4Direct
                              ? "Q-STAND-HOLD"
                              : "KEY3_LAST_SAFE_COMMAND";
            publish_ok = publish_hold_with_raw_stats(
                key3_phase == Key3Phase::kLocomotionGait4Direct
                    ? q_hold_command
                    : key3_last_safe_command);
            mode = Mode::kQHold;
            key3_phase = Key3Phase::kNone;
            gait0_raw_released = false;
          }
        } else {
          apply_gait0_blend_softening();
          if (!apply_guarded_raw()) {
          if (guard.fault_reason == "low_kp_kd") {
            std::cerr << "[LEGBOT-DDS][SOFT-FAULT] low_kp_kd; back to Q-STAND-HOLD. Press 4 to exit.\n";
            mode = Mode::kQHold;
            key3_phase = Key3Phase::kNone;
            gait0_raw_released = false;
            previous_q_des = legbot::StandQDds();
            previous_qd_des.fill(0.);
            has_previous_q_des = true;
            continue;
          }
          std::cerr << "[LEGBOT-DDS][HARD-FAULT] " << guard.fault_reason
                    << "\n";
          PublishCurrentQDampingHoldThenDisable(args, snapshot, lowcmd_pub,
                                                command_adapter, stats,
                                                stats_t0);
          return std::string("runtime_safety_failed:") + guard.fault_reason;
          }
          publish_tag = "RAW_GUARDED";
          if (key3_phase == Key3Phase::kGait0Blend) publish_tag = "GAIT0_BLEND_SOFT";
          const bool gait0_output_phase =
              key3_phase == Key3Phase::kGait0Blend ||
              key3_phase == Key3Phase::kGait0Direct;
          if (gait0_output_phase) {
            if (guard.num_qdes_clamped > 0) {
              ++gait0_monitor_qdes_clamped_count;
            } else {
              gait0_monitor_qdes_clamped_count = 0;
            }
            double entry_limit = std::numeric_limits<double>::infinity();
            if (gait0_entry_violation.joint >= 0) {
              entry_limit = ReadyMaxQErrorForDdsJoint(
                  gait0_entry_violation.joint,
                  true /* gait0_shadow_entry_check */);
            }
            if (gait0_entry_violation.joint >= 0 &&
                gait0_entry_violation.value > entry_limit) {
              ++gait0_monitor_entry_violation_count;
            } else {
              gait0_monitor_entry_violation_count = 0;
            }
            const double monitor_dq_feedback = MaxAbsDqFeedback(snapshot);
            const double monitor_tilt = state_adapter->Tilt(snapshot);
            std::ostringstream monitor_abort;
            if (guard.num_tau_clamped > 0) {
              monitor_abort << "tau_clamped num_tau_clamped="
                            << guard.num_tau_clamped;
            } else if (gait0_monitor_qdes_clamped_count >=
                       kGait0BlendMonitorClampFrames) {
              monitor_abort << "qdes_clamped_consecutive count="
                            << gait0_monitor_qdes_clamped_count
                            << " num_qdes_clamped="
                            << guard.num_qdes_clamped;
            } else if (!std::isfinite(monitor_dq_feedback) ||
                       monitor_dq_feedback >
                           kGait0BlendMonitorMaxAbsDqFeedback) {
              monitor_abort << "dq_feedback_too_large max_abs_dq_feedback="
                            << monitor_dq_feedback
                            << " limit="
                            << kGait0BlendMonitorMaxAbsDqFeedback;
            } else if (!std::isfinite(monitor_tilt) ||
                       monitor_tilt > kGait0BlendMonitorMaxTilt) {
              monitor_abort << "tilt_too_large tilt=" << monitor_tilt
                            << " limit=" << kGait0BlendMonitorMaxTilt;
            } else if (key3_phase != Key3Phase::kGait0Blend &&
                       gait0_monitor_entry_violation_count >=
                       kGait0BlendMonitorClampFrames) {
              monitor_abort << "entry_envelope_violation_consecutive count="
                            << gait0_monitor_entry_violation_count
                            << " joint="
                            << legbot::JointNameDds(
                                   gait0_entry_violation.joint)
                            << " value=" << gait0_entry_violation.value
                            << " limit=" << entry_limit;
            }
            if (key3_phase == Key3Phase::kGait0Blend &&
                gait0_blend_entry_violation) {
              std::cout << "[GAIT0-BLEND-MONITOR] warn reason="
                        << (gait0_blend_entry_soft_hold
                                ? "entry_envelope_violation_soft"
                                : "entry_envelope_violation_warn_only")
                        << " entry_envelope_mode="
                        << gait0_blend_entry_envelope_mode
                        << " entry_violation_joint="
                        << (gait0_entry_violation.joint >= 0
                                ? legbot::JointNameDds(
                                      gait0_entry_violation.joint)
                                : "unknown")
                        << " entry_violation_value="
                        << gait0_entry_violation.value
                        << " entry_violation_limit="
                        << gait0_blend_entry_limit
                        << " entry_violation_count="
                        << gait0_monitor_entry_violation_count
                        << " alpha_q=" << alpha_q
                        << " alpha_tau=" << alpha_tau
                        << " alpha_frozen="
                        << (gait0_blend_entry_soft_hold ? "true" : "false")
                        << " abort_reason=none"
                        << " publish=GAIT0_BLEND_SOFT"
                        << " requested_gait=" << requested_gait
                        << " max_abs_dq_feedback=" << monitor_dq_feedback
                        << " tilt=" << monitor_tilt
                        << " max_raw_q_step=" << gait0_raw_q_step.value
                        << "\n";
            }
            if (!monitor_abort.str().empty()) {
              std::ostringstream abort_detail;
              abort_detail << "entry_envelope_mode="
                           << gait0_blend_entry_envelope_mode
                           << " entry_violation_joint="
                           << (gait0_entry_violation.joint >= 0
                                   ? legbot::JointNameDds(
                                         gait0_entry_violation.joint)
                                   : "unknown")
                           << " entry_violation_value="
                           << gait0_entry_violation.value
                           << " entry_violation_limit="
                           << gait0_blend_entry_limit
                           << " entry_violation_count="
                           << gait0_monitor_entry_violation_count
                           << " alpha_q=" << alpha_q
                           << " alpha_tau=" << alpha_tau
                           << " alpha_frozen="
                           << (gait0_blend_entry_soft_hold ? "true" : "false")
                           << " abort_reason=" << monitor_abort.str()
                           << " max_abs_dq_feedback=" << monitor_dq_feedback
                           << " tilt=" << monitor_tilt
                           << " max_raw_q_step=" << gait0_raw_q_step.value;
              publish_ok =
                  abort_gait0_to_qstand(monitor_abort.str(),
                                        abort_detail.str());
              if (!publish_ok) {
                std::cerr << "[LEGBOT-DDS][ERROR] publish_failed during gait0-monitor-abort\n";
              }
              goto key3_after_publish_decision;
            }
          } else {
            gait0_monitor_qdes_clamped_count = 0;
            gait0_monitor_entry_violation_count = 0;
          }
          publish_ok = command_adapter->PublishLowCmd(lowcmd_pub, guard.pub,
                                                      stats, stats_t0);
          key3_last_safe_command = guard.pub;
          previous_q_des = guard.pub.q;
          previous_qd_des = guard.pub.dq;
          has_previous_q_des = true;

          if (key3_phase == Key3Phase::kLocomotionGait4Direct &&
              phase_elapsed >= kKey3Gait4DirectSeconds &&
              std::abs(args.test_gait - 4.0) > 1.e-6) {
            const double qerr_to_pub =
                MaxAbsPublishedQFeedbackDiff(guard.pub, snapshot);
            const double tilt = state_adapter->Tilt(snapshot);
            const double max_abs_dq_feedback = MaxAbsDqFeedback(snapshot);
            bool gait4_direct_stable = true;
            std::ostringstream wait_reason;
            if (!std::isfinite(qerr_to_pub) ||
                qerr_to_pub > kKey3Gait4ToGait0MaxAbsQError) {
              gait4_direct_stable = false;
              wait_reason << "qerr_to_pub=" << qerr_to_pub
                          << " limit=" << kKey3Gait4ToGait0MaxAbsQError;
            } else if (!std::isfinite(tilt) ||
                       tilt > kKey3Gait4ToGait0MaxTilt) {
              gait4_direct_stable = false;
              wait_reason << "tilt=" << tilt
                          << " limit=" << kKey3Gait4ToGait0MaxTilt;
            } else if (!std::isfinite(max_abs_dq_feedback) ||
                       max_abs_dq_feedback >
                           kKey3Gait4ToGait0MaxAbsDqFeedback) {
              gait4_direct_stable = false;
              wait_reason << "max_abs_dq_feedback="
                          << max_abs_dq_feedback
                          << " limit="
                          << kKey3Gait4ToGait0MaxAbsDqFeedback;
            }

            if (gait4_direct_stable) {
              key3_last_safe_command = guard.pub;
              if (controller) controller->resetConvexMPCGaitPhase();
              gait0_entry_q_fb = snapshot.dds_q;
              gait0_entry_q_hold = key3_last_safe_command.q;
              gait0_raw_released = false;
              std::cout << "[LEGBOT-DDS][KEY3-PHASE] reset ConvexMPC gait phase before GAIT0_SHADOW"
                        << " firstSwing=reset swingTimeRemaining=0"
                        << " qerr_to_pub=" << qerr_to_pub
                        << " tilt=" << tilt
                        << " max_abs_dq_feedback="
                        << max_abs_dq_feedback
                        << " gait0_entry_q_fb="
                        << legbot::ArrayText(gait0_entry_q_fb, 4)
                        << " gait0_entry_q_hold="
                        << legbot::ArrayText(gait0_entry_q_hold, 4)
                        << "\n";
              set_key3_phase(Key3Phase::kGait0Shadow,
                             "gait4_direct_stable");
            } else {
              const std::string reason = wait_reason.str();
              if (reason != last_gait4_to_gait0_wait_reason) {
                last_gait4_to_gait0_wait_reason = reason;
                std::cout << "[LEGBOT-DDS][KEY3-PHASE] hold in LOCOMOTION_GAIT4_DIRECT before GAIT0_SHADOW"
                          << " reason=" << reason
                          << " elapsed_in_phase=" << std::fixed
                          << std::setprecision(3) << phase_elapsed
                          << "\n";
              }
            }
          } else if (key3_phase == Key3Phase::kGait0Blend &&
                     phase_elapsed >= kKey3BlendTauSeconds) {
            set_key3_phase(Key3Phase::kGait0Direct, "gait0_blend_complete");
          }
        }
      }

key3_after_publish_decision:
      if (!publish_ok) {
        std::cerr << "[LEGBOT-DDS][ERROR] publish_failed during key3-locomotion\n";
        command_adapter->DisableBurst(lowcmd_pub, args, stats, stats_t0);
        return "publish_failed:key3-locomotion";
      }

      csv_logger.MaybeWrite(now, "KEY3_LOCOMOTION", requested_gait,
                            user_params, robot_params, snapshot,
                            *state_adapter, guard, runner, controller,
                            bridge_ready.ready, bridge_ready.reject_code,
                            Key3PhaseText(key3_phase), alpha_q, alpha_qd,
                            alpha_tau);

      if (now >= next_print) {
        const double max_abs_qd_des_pub = MaxAbsPublishedQd(guard.pub);
        const double max_abs_dq_feedback = MaxAbsDqFeedback(snapshot);
        const bool dq_feedback_too_large =
            !std::isfinite(max_abs_dq_feedback) ||
            max_abs_dq_feedback > kGait0BlendMonitorMaxAbsDqFeedback;
        PrintDirectAlgoLog("KEY3-LOCOMOTION", Key3PhaseText(key3_phase),
                           phase_elapsed, kKey3Gait4DirectSeconds, 0.0,
                           requested_gait, user_params, snapshot,
                           *state_adapter, guard, runner, controller);
        std::cout << "bridge_ready=" << (bridge_ready.ready ? "true" : "false")
                  << " bridge_reject_code="
                  << BridgeRejectCodeText(bridge_ready.reject_code)
                  << " reason=" << bridge_log_reason
                  << " publish=" << publish_tag
                  << " max_abs_axis_jump=" << foot_jump_diagnostic
                  << " foot_jump_diagnostic=" << foot_jump_diagnostic
                  << " leg_command_jump="
                  << leg_command_foot_jump_diagnostic
                  << " foot_jump_warn_limit=" << kFootTargetJumpWarnLimit
                  << " foot_jump_hard_limit="
                  << foot_target_jump_hard_limit
                  << " foot_jump_hard_gate="
                  << (foot_jump_hard_gate ? "true" : "false")
                  << " wbc_sum_fz=" << wbc_sum_fz_diagnostic
                  << " cmpc_sum_fz=" << cmpc_sum_fz_diagnostic
                  << " leg_command_sum_fz="
                  << leg_command_sum_fz_diagnostic
                  << " wbc_force_warn_min=" << kWbcForceWarnMinFz
                  << " wbc_force_hard_min=" << wbc_force_hard_min
                  << " wbc_force_hard_gate="
                  << (wbc_force_hard_gate ? "true" : "false")
                  << " gait0_compute_only="
                  << (gait0_compute_only ? "true" : "false")
                  << " gait0_raw_released="
                  << (gait0_raw_released ? "true" : "false")
                  << " gait0_raw_stable_count="
                  << gait0_raw_stable_count
                  << " gait0_window_elapsed="
                  << legbot::SecondsSince(gait0_window_start, now)
                  << " gait0_window_mpc_table_transitions="
                  << gait0_window_mpc_table_transitions
                  << " gait0_window_max_raw_q_step="
                  << gait0_window_max_raw_q_step
                  << " gait0_entry_violation_increase_count="
                  << gait0_entry_violation_increase_count
                  << " max_entry_violation="
                  << gait0_entry_violation.value;
        if (gait0_entry_violation.joint >= 0) {
          std::cout << " max_entry_violation_joint="
                    << legbot::JointNameDds(gait0_entry_violation.joint);
        }
        std::cout << " max_raw_q_step=" << gait0_raw_q_step.value;
        if (gait0_raw_q_step.joint >= 0) {
          std::cout << " max_raw_q_step_joint="
                    << legbot::JointNameDds(gait0_raw_q_step.joint);
        }
        std::cout << " max_abs_raw_qd=" << gait0_max_abs_raw_qd
                  << " max_abs_raw_tau=" << gait0_max_abs_raw_tau
                  << " raw_release_block_reason="
                  << gait0_raw_release_block_reason
                  << " alpha_q=" << alpha_q << " alpha_qd=" << alpha_qd
                  << " alpha_tau=" << alpha_tau
                  << " qd_des_raw=" << legbot::ArrayText(guard.raw.dq, 4)
                  << " qd_des_pub=" << legbot::ArrayText(guard.pub.dq, 4)
                  << " max_abs_qd_des_pub=" << max_abs_qd_des_pub
                  << " max_abs_dq_feedback=" << max_abs_dq_feedback
                  << " dq_feedback_too_large="
                  << (dq_feedback_too_large ? "true" : "false")
                  << " num_qd_delta_clamped="
                  << guard.num_qd_delta_clamped
                  << " gait0_blend_q_seconds=" << kKey3BlendQSeconds
                  << " gait0_blend_alpha_step_limit="
                  << kGait0BlendMaxAlphaStep
                  << " gait0_blend_q_step_limit="
                  << kGait0BlendMaxQStep
                  << " gait0_blend_tau_alpha_scale="
                  << kGait0BlendTauAlphaScale
                  << " gait0_blend_qd_alpha="
                  << kGait0BlendQdAlpha
                  << " gait0_blend_alpha_scale_hip="
                  << kGait0BlendHipAlphaScale
                  << " gait0_blend_alpha_scale_thigh="
                  << kGait0BlendThighAlphaScale
                  << " gait0_blend_alpha_scale_calf="
                  << kGait0BlendCalfAlphaScale
                  << " gait0_blend_q_step_hip="
                  << kGait0BlendHipMaxQStep
                  << " gait0_blend_q_step_thigh="
                  << kGait0BlendThighMaxQStep
                  << " gait0_blend_q_step_calf="
                  << kGait0BlendCalfMaxQStep
                  << " gait0_blend_tau_alpha_hip="
                  << kGait0BlendHipTauAlphaScale
                  << " gait0_blend_tau_alpha_thigh="
                  << kGait0BlendThighTauAlphaScale
                  << " gait0_blend_tau_alpha_calf="
                  << kGait0BlendCalfTauAlphaScale
                  << " gait0_blend_stance_kp_min=["
                  << kGait0BlendStanceHipKpMin << ','
                  << kGait0BlendStanceThighKpMin << ','
                  << kGait0BlendStanceCalfKpMin << ']'
                  << " gait0_blend_stance_kd_min=["
                  << kGait0BlendStanceHipKdMin << ','
                  << kGait0BlendStanceThighKdMin << ','
                  << kGait0BlendStanceCalfKdMin << ']'
                  << "\n";
        next_print = now + std::chrono::duration_cast<legbot::Clock::duration>(
                                  std::chrono::duration<double>(0.2));
      }
    } else {
      state_adapter->FillCheetahData(snapshot, spi_data, vector_nav_data);
      FillCheaterFromLowState(snapshot, cheater_state);

      const double requested_gait = 4.0;
      const double requested_vx = 0.0;
      ApplyControllerCommand(ControllerPhase::kBalanceStand,
                             requested_gait, requested_vx, 0., 0.,
                             robot_params, user_params, gamepad_command);
      runner->run();

      auto raw_command = command_adapter->FromSpiCommand(*spi_command);
      command_adapter->ApplyTauScale(args.tau_ff_scale, &raw_command);
      legbot::OutputGuardStats guard;
      MitOutputReadiness bridge_ready;
      bridge_ready.ready = true;
      bridge_ready.reason = "ok";

      if (args.real_output_raw) {
        if (!PopulateRawCommandStats(raw_command, &guard)) {
          std::cerr << "[LEGBOT-DDS][HARD-FAULT] " << guard.fault_reason << "\n";
          PublishCurrentQDampingHoldThenDisable(args, snapshot, lowcmd_pub,
                                                command_adapter, stats,
                                                stats_t0);
          return std::string("runtime_safety_failed:") + guard.fault_reason;
        }
      } else {
        if (!legbot::ApplyOutputGuard(
                raw_command, args,
                has_previous_q_des ? &previous_q_des : nullptr,
                has_previous_q_des ? &previous_qd_des : nullptr, &guard)) {
          if (guard.fault_reason == "low_kp_kd") {
            std::cerr << "[LEGBOT-DDS][SOFT-FAULT] low_kp_kd; stop MIT output, back to Q-STAND-HOLD. Press 4 to exit.\n";
            mode = Mode::kQHold;
            previous_q_des = legbot::StandQDds();
            previous_qd_des.fill(0.);
            has_previous_q_des = true;
            continue;
          }
          std::cerr << "[LEGBOT-DDS][HARD-FAULT] " << guard.fault_reason << "\n";
          PublishCurrentQDampingHoldThenDisable(args, snapshot, lowcmd_pub,
                                                command_adapter, stats,
                                                stats_t0);
          return std::string("runtime_safety_failed:") + guard.fault_reason;
        }
      }
      if (!command_adapter->PublishLowCmd(lowcmd_pub, guard.pub, stats,
                                          stats_t0)) {
        std::cerr << "[LEGBOT-DDS][ERROR] publish_failed during interactive-algo-direct\n";
        command_adapter->DisableBurst(lowcmd_pub, args, stats, stats_t0);
        return "publish_failed:interactive-algo-direct";
      }
      csv_logger.MaybeWrite(
          now, "MIT_BALANCE_STAND_DIRECT",
          requested_gait, user_params, robot_params, snapshot,
          *state_adapter, guard, runner, controller, bridge_ready.ready,
          bridge_ready.reject_code, "DIRECT");
      previous_q_des = guard.pub.q;
      previous_qd_des = guard.pub.dq;
      has_previous_q_des = true;

      if (now >= next_print) {
        PrintDirectAlgoLog("MIT-BALANCE-STAND-DIRECT",
                           nullptr, 0., 0., requested_vx,
                           requested_gait, user_params,
                           snapshot, *state_adapter, guard, runner,
                           controller);
        next_print = now + std::chrono::duration_cast<legbot::Clock::duration>(
                                  std::chrono::duration<double>(0.2));
      }
    }

    ++stats->loop_count;
    const double loop_ms =
        std::chrono::duration<double, std::milli>(legbot::Clock::now() - loop_t0)
            .count();
    stats->loop_ms_sum += loop_ms;
    stats->loop_ms_max = std::max(stats->loop_ms_max, loop_ms);
    next += std::chrono::duration_cast<legbot::Clock::duration>(
        std::chrono::duration<double>(period_s));
    std::this_thread::sleep_until(next);
  }
  return g_running ? "interactive_control_completed" : "signal";
}

void RunDryRun(const legbot::BridgeArgs& args,
               const legbot::LegBotDDSStateAdapter& state_adapter,
               legbot::LoopStats* stats,
               const legbot::Clock::time_point& stats_t0) {
  std::string wait_reason;
  if (!state_adapter.WaitForAnyLowState(args, &wait_reason)) {
    std::cerr << "[LEGBOT-DDS][ERROR] " << wait_reason
              << " before dry-run\n";
    return;
  }
  std::cout << "[LEGBOT-DDS] LowState received. Stage 0 dry-run does not create "
               "a LowCmd publisher.\n";

  const auto start = legbot::Clock::now();
  auto next = start;
  auto next_print = start;
  auto last_sample_t = start;
  uint32_t last_tick = std::numeric_limits<uint32_t>::max();
  const double period_s = 1. / args.cmd_hz;

  while (g_running) {
    const auto now = legbot::Clock::now();
    const double elapsed = legbot::SecondsSince(start, now);
    if (elapsed >= args.duration_s) break;
    if (state_adapter.IsTimeout()) {
      throw std::runtime_error("rt_lowstate_timeout during dry-run");
    }

    const auto loop_t0 = legbot::Clock::now();
    const auto snapshot = state_adapter.Snapshot();
    legbot::RecordLowStateTiming(snapshot.tick, stats, &last_sample_t,
                               &last_tick);
    stats->max_tilt = std::max(stats->max_tilt, state_adapter.Tilt(snapshot));
    ++stats->loop_count;

    if (now >= next_print || stats->loop_count == 1) {
      PrintLowState(snapshot, state_adapter, *stats,
                    legbot::SecondsSince(stats_t0, now));
      next_print = now + std::chrono::seconds(1);
    }

    const double loop_ms =
        std::chrono::duration<double, std::milli>(legbot::Clock::now() - loop_t0)
            .count();
    stats->loop_ms_sum += loop_ms;
    stats->loop_ms_max = std::max(stats->loop_ms_max, loop_ms);
    next += std::chrono::duration_cast<legbot::Clock::duration>(
        std::chrono::duration<double>(period_s));
    std::this_thread::sleep_until(next);
  }
}

std::string RunControllerOutput(
    const legbot::BridgeArgs& args,
    const std::shared_ptr<legbot::LowStateSubscriber>& sub,
  legbot::LowCmdPublisher* lowcmd_pub, legbot::LegBotDDSStateAdapter* state_adapter,
  legbot::LoopStats* stats, const legbot::Clock::time_point& stats_t0) {
  const bool real_output = RealOutputEnabled(args);
  std::string wait_reason;
  if (real_output) {
    if (!state_adapter->WaitForAnyLowState(args, &wait_reason)) {
      std::cerr << "[LEGBOT-DDS][ERROR] " << wait_reason
                << " before real controller-output\n";
      return "lowstate_wait_failed";
    }
  } else {
    if (!state_adapter->WaitForValidLowState(args, &wait_reason)) {
      std::cerr << "[LEGBOT-DDS][ERROR] " << wait_reason
                << " before dry-output/controller-output\n";
      return "lowstate_wait_failed";
    }
  }
  if (args.real_output_guarded) {
    std::cout << "[LEGBOT-DDS] LowState valid. Guarded mode will publish "
              << kLowCmdTopic << " after feedback and OutputGuard checks.\n";
  } else if (args.real_output_raw) {
    std::cout << "[LEGBOT-DDS] LowState valid. RAW mode will publish "
              << kLowCmdTopic
              << " without OutputGuard command clipping; feedback safety remains active.\n";
  } else {
    std::cout << "[LEGBOT-DDS] LowState valid. Stage 1 dry-output runs "
                 "RobotRunner but never publishes LowCmd.\n";
  }

  RobotControlParameters robot_params;
  robot_params.initializeFromYamlFile(THIS_COM "config/legbot-defaults.yaml");
  if (!robot_params.isFullyInitialized()) {
    std::cerr << "[LEGBOT-DDS][ERROR] robot parameters are not fully initialized\n";
    return "robot_params_init_failed";
  }
  // LegBot DDS real bridge must use the real-robot estimator path. Cheetah
  // Cheater Mode consumes simulation truth and is not valid for DDS-only I/O.
  robot_params.cheater_mode = 0;
  robot_params.use_rc = 0;
  robot_params.controller_dt = 1.0f / static_cast<float>(args.cmd_hz);
  robot_params.control_mode = kPassiveControlMode;
  std::cout << "[LEGBOT-DDS] robot_params.cheater_mode="
            << robot_params.cheater_mode
            << " controller_dt=" << robot_params.controller_dt
            << " use_rc=" << robot_params.use_rc << "\n";

  MIT_Controller controller;
  ControlParameters* user_params = controller.getUserControlParameters();
  if (!user_params) {
    std::cerr << "[LEGBOT-DDS][ERROR] MIT_Controller did not expose user parameters\n";
    return "mit_user_params_missing";
  }
  user_params->initializeFromYamlFile(
      THIS_COM "config/legbot-mit-ctrl-user-parameters.yaml");
  if (!user_params->isFullyInitialized()) {
    std::cerr << "[LEGBOT-DDS][ERROR] MIT user parameters are not fully initialized\n";
    return "mit_user_params_init_failed";
  }
  auto* mit_user_params = static_cast<MIT_UserParameters*>(user_params);
  if (args.test_locomotion) {
    std::cout << "[LEGBOT-DDS] Test locomotion enabled: gait="
              << args.test_gait << " vx=" << args.test_vx
              << " vy=" << args.test_vy
              << " yaw_rate=" << args.test_yaw_rate << "\n";
  }

  PeriodicTaskManager task_manager;
  RobotRunner runner(&controller, &task_manager, robot_params.controller_dt,
                     "legbot-dry-output-robot-runner");
  GamepadCommand gamepad_command;
  SpiData spi_data;
  SpiCommand spi_command;
  VectorNavData vector_nav_data;
  CheaterState<double> cheater_state;
  VisualizationData visualization_data;
  CheetahVisualization main_visualization;
  std::memset(&spi_data, 0, sizeof(spi_data));
  std::memset(&spi_command, 0, sizeof(spi_command));
  vector_nav_data.quat << 1.f, 0.f, 0.f, 0.f;
  vector_nav_data.gyro.setZero();
  vector_nav_data.accelerometer << 0.f, 0.f, 9.81f;
  cheater_state.orientation << 1., 0., 0., 0.;
  cheater_state.position << 0., 0., kLegBotNominalHeight;
  cheater_state.omegaBody.setZero();
  cheater_state.vBody.setZero();
  cheater_state.acceleration.setZero();

  runner.driverCommand = &gamepad_command;
  runner.robotType = RobotType::LEGBOT;
  runner.vectorNavData = &vector_nav_data;
  runner.cheaterState = &cheater_state;
  runner.spiData = &spi_data;
  runner.spiCommand = &spi_command;
  runner.controlParameters = &robot_params;
  runner.visualizationData = &visualization_data;
  runner.cheetahMainVisualization = &main_visualization;
  runner.init();
  runner.initializeStateEstimator(false);

  legbot::LegBotDDSCommandAdapter command_adapter;
  if (real_output) {
    if (!BootstrapMotorFeedback(args, state_adapter, lowcmd_pub,
                                &command_adapter, stats, stats_t0)) {
      return "feedback_bootstrap_failed";
    }
    if (args.joint_nudge_test) {
      return RunJointNudgeTest(args, state_adapter, lowcmd_pub,
                               &command_adapter, stats, stats_t0);
    }
    if (args.interactive_control) {
      return RunInteractiveControl(
          args, &controller, &runner, &robot_params, mit_user_params, &gamepad_command,
          &spi_data, &spi_command, &vector_nav_data, &cheater_state,
          state_adapter, lowcmd_pub, &command_adapter, stats, stats_t0);
    }
    RunGuardedStandup(args, state_adapter, lowcmd_pub, &command_adapter, stats,
                      stats_t0);
  }
  const auto start = legbot::Clock::now();
  auto next = start;
  auto next_print = start;
  auto last_sample_t = start;
  auto last_tick_change_t = start;
  uint32_t last_lowstate_tick = std::numeric_limits<uint32_t>::max();
  uint32_t last_recorded_tick = std::numeric_limits<uint32_t>::max();
  std::array<double, legbot::kNumJoints> previous_q_des{};
  std::array<double, legbot::kNumJoints> previous_qd_des{};
  bool has_previous_q_des = false;
  bool has_valid_controller_command = false;
  std::string exit_reason =
      args.real_output_guarded
          ? "real_output_guarded_completed"
          : (args.real_output_raw ? "real_output_raw_completed"
                                  : "dry_output_completed");
  const double period_s = 1. / args.cmd_hz;
  const double fsm_warmup_s = real_output ? args.fsm_warmup_seconds : 0.;
  const double controller_stand_s =
      real_output ? args.standup_prehold_seconds : 0.;
  const double controller_total_s =
      fsm_warmup_s + controller_stand_s + args.duration_s;

  try {
    while (g_running) {
      const auto now = legbot::Clock::now();
      const double elapsed = legbot::SecondsSince(start, now);
      if (elapsed >= controller_total_s) break;
      const bool fsm_warmup = elapsed < fsm_warmup_s;
      const bool controller_stand =
          !fsm_warmup && elapsed < fsm_warmup_s + controller_stand_s;
      const double phase_elapsed =
          fsm_warmup
              ? elapsed
              : (controller_stand ? elapsed - fsm_warmup_s
                                  : elapsed - fsm_warmup_s - controller_stand_s);
      const ControllerPhase phase =
          fsm_warmup ? ControllerPhase::kFsmWarmup
                     : (controller_stand ? ControllerPhase::kControllerStand
                                         : ControllerPhase::kLocomotionTest);
      if (state_adapter->IsTimeout()) {
        const auto snapshot = state_adapter->Snapshot();
        std::cerr << "[LEGBOT-DDS][HARD-FAULT] rt_lowstate_timeout during controller-output\n";
        if (real_output) {
          PublishCurrentQDampingHoldThenDisable(args, snapshot, lowcmd_pub,
                                                &command_adapter, stats,
                                                stats_t0);
        }
        exit_reason = "runtime_safety_failed:rt_lowstate_timeout";
        break;
      }

      const auto loop_t0 = legbot::Clock::now();
      const auto snapshot = state_adapter->Snapshot();
      const auto tick_safety = CheckTickAdvances(
          snapshot, args, now, &last_lowstate_tick, &last_tick_change_t);
      if (!tick_safety.ok) {
        std::cerr << "[LEGBOT-DDS][HARD-FAULT] " << tick_safety.reason << "\n";
        if (real_output) {
          PublishCurrentQDampingHoldThenDisable(args, snapshot, lowcmd_pub,
                                                &command_adapter, stats,
                                                stats_t0);
        }
        exit_reason = std::string("runtime_safety_failed:") +
                      tick_safety.reason;
        break;
      }
      legbot::RecordLowStateTiming(snapshot.tick, stats, &last_sample_t,
                                 &last_recorded_tick);
      if (real_output) {
        const auto lowstate_safety =
            CheckInteractiveLowState(snapshot, *state_adapter, args, stats);
        if (!lowstate_safety.ok) {
          if (lowstate_safety.reason.find("runtime_joint_limit:") == 0) {
            const auto limit = legbot::FeedbackQLimitForModelJoint(
                legbot::DdsToModelJoint(lowstate_safety.joint), args);
            std::cerr << "[LEGBOT-DDS][JOINT-LIMIT-FAULT]\n"
                      << "joint=" << legbot::JointNameDds(lowstate_safety.joint)
                      << "\nindex=" << lowstate_safety.joint
                      << "\nq=" << lowstate_safety.value
                      << "\nlimit=[" << limit.first << "," << limit.second
                      << "]\n";
            exit_reason =
                std::string("runtime_safety_failed:") +
                lowstate_safety.reason;
          } else {
            std::cerr << "[LEGBOT-DDS][HARD-FAULT] "
                      << lowstate_safety.reason
                      << " value=" << lowstate_safety.value << "\n";
            exit_reason =
                std::string("runtime_safety_failed:") +
                lowstate_safety.reason;
          }
          PublishCurrentQDampingHoldThenDisable(args, snapshot, lowcmd_pub,
                                                &command_adapter, stats,
                                                stats_t0);
          break;
        }
      }
      state_adapter->FillCheetahData(snapshot, &spi_data, &vector_nav_data);
      FillCheaterFromLowState(snapshot, &cheater_state);
      ApplyControllerTestCommand(args, phase, &robot_params,
                                 mit_user_params, &gamepad_command,
                                 phase_elapsed);

      runner.run();

      auto command = command_adapter.FromSpiCommand(spi_command);
      command_adapter.ApplyTauScale(args.tau_ff_scale, &command);
      has_valid_controller_command =
          has_valid_controller_command || HasControllerCommand(command);
      const auto model_q_target = has_valid_controller_command
                                      ? ModelQTargetFromDdsCommand(command)
                                      : snapshot.model_q;
      legbot::OutputGuardStats guard;
      if (args.real_output_guarded) {
        if (!legbot::ApplyOutputGuard(
                command, args, has_previous_q_des ? &previous_q_des : nullptr,
                has_previous_q_des ? &previous_qd_des : nullptr,
                &guard)) {
          std::cerr << "[LEGBOT-DDS][HARD-FAULT] " << guard.fault_reason
                    << "\n";
          PublishCurrentQDampingHoldThenDisable(args, snapshot, lowcmd_pub,
                                                &command_adapter, stats,
                                                stats_t0);
          exit_reason = std::string("runtime_safety_failed:") +
                        guard.fault_reason;
          break;
        }
        if (!command_adapter.PublishLowCmd(lowcmd_pub, guard.pub, stats,
                                           stats_t0)) {
          std::cerr << "[LEGBOT-DDS][ERROR] publish_failed during real-output-guarded\n";
          command_adapter.DisableBurst(lowcmd_pub, args, stats, stats_t0);
          exit_reason = "publish_failed:real-output-guarded";
          break;
        }
        previous_q_des = guard.pub.q;
        previous_qd_des = guard.pub.dq;
        has_previous_q_des = true;
      } else if (args.real_output_raw) {
        if (!PopulateRawCommandStats(command, &guard)) {
          std::cerr << "[LEGBOT-DDS][HARD-FAULT] " << guard.fault_reason
                    << "\n";
          PublishCurrentQDampingHoldThenDisable(args, snapshot, lowcmd_pub,
                                                &command_adapter, stats,
                                                stats_t0);
          exit_reason = std::string("runtime_safety_failed:") +
                        guard.fault_reason;
          break;
        }
        if (!command_adapter.PublishLowCmd(lowcmd_pub, guard.pub, stats,
                                           stats_t0)) {
          std::cerr << "[LEGBOT-DDS][ERROR] publish_failed during real-output-raw\n";
          command_adapter.DisableBurst(lowcmd_pub, args, stats, stats_t0);
          exit_reason = "publish_failed:real-output-raw";
          break;
        }
      } else {
        const bool enable_qerr_check = false;
        const auto safety = state_adapter->CheckRuntimeSafety(
            snapshot, has_valid_controller_command ? &model_q_target : nullptr,
            enable_qerr_check,
            args, stats);
        if (!safety.ok) {
          std::cerr << "[LEGBOT-DDS][SAFETY] " << safety.reason;
          if (safety.joint >= 0) {
            std::cerr << " joint=" << safety.joint;
          }
          std::cerr << " value=" << safety.value << "\n";
          exit_reason = "runtime_safety_failed";
          break;
        }
        guard.raw = command;
        guard.pub = command;
      }

      if (now >= next_print || stats->loop_count == 0) {
        PrintLowState(snapshot, *state_adapter, *stats,
                      legbot::SecondsSince(stats_t0, now));
        const char* tag = "DRY-OUTPUT";
        if (args.real_output_guarded) {
          tag = fsm_warmup ? "FSM-WARMUP"
                           : (controller_stand ? "CONTROLLER-STAND"
                                               : "REAL-OUTPUT-GUARDED");
        } else if (args.real_output_raw) {
          tag = fsm_warmup ? "FSM-WARMUP-RAW"
                           : (controller_stand ? "CONTROLLER-STAND-RAW"
                                               : "REAL-OUTPUT-RAW");
        }
        PrintCommandSummary(real_output ? guard.pub : command,
                            phase_elapsed, tag);
        next_print = now + std::chrono::seconds(1);
      }

      ++stats->loop_count;
      const double loop_ms =
          std::chrono::duration<double, std::milli>(legbot::Clock::now() - loop_t0)
              .count();
      stats->loop_ms_sum += loop_ms;
      stats->loop_ms_max = std::max(stats->loop_ms_max, loop_ms);
      next += std::chrono::duration_cast<legbot::Clock::duration>(
          std::chrono::duration<double>(period_s));
      std::this_thread::sleep_until(next);
    }
  } catch (const std::exception& e) {
    if (real_output && g_running) {
      std::cerr << "[LEGBOT-DDS][HARD-FAULT] unexpected controller exception: "
                << e.what() << "; disable without prone ramp.\n";
      const auto snapshot = state_adapter->Snapshot();
      PublishCurrentQDampingHoldThenDisable(args, snapshot, lowcmd_pub,
                                            &command_adapter, stats, stats_t0);
      exit_reason = std::string("runtime_safety_failed:") + e.what();
    } else {
      exit_reason = std::string("exception:") + e.what();
    }
  }

  if (real_output && g_running &&
      (exit_reason == "real_output_guarded_completed" ||
       exit_reason == "real_output_raw_completed")) {
    RunGuardedRampDown(args, legbot::DownQDds(), state_adapter, lowcmd_pub,
                       &command_adapter, stats, stats_t0);
  }

  (void)sub;
  return g_running ? exit_reason : "signal";
}

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, OnSignal);
  std::signal(SIGTERM, OnSignal);

  legbot::BridgeArgs args;
  legbot::LoopStats stats;
  const auto stats_t0 = legbot::Clock::now();
  std::string exit_reason = "not_started";
  std::unique_ptr<legbot::LowCmdPublisher> lowcmd_pub;
  legbot::LegBotDDSCommandAdapter exit_command_adapter;

  try {
    args = legbot::ParseArgs(argc, argv);
    if (args.help) {
      legbot::PrintHelp(argv[0]);
      return 0;
    }
    legbot::ValidateArgs(args);

    std::cout << "[LEGBOT-DDS] build features: "
              << "key1=Q_STAND, "
              << "key2=MIT_BALANCE_STAND_DIRECT, "
              << "key3=LOCOMOTION_SHADOW_BLEND_TAKEOVER, "
              << "safety=OUTPUT_GUARD_OR_RAW+FEEDBACK_LIMIT+IMU_DEADZONE, "
              << "alpha_q_qd_tau\n";

    std::cout << "[LEGBOT-DDS] stage=" << args.stage
              << " network=" << args.network << " cmd_hz=" << args.cmd_hz
              << " duration=" << args.duration_s
              << "; subscribe " << kLowStateTopic;
    if (args.real_output_guarded) {
      std::cout << "; publish " << kLowCmdTopic << " guarded\n";
    } else if (args.real_output_raw) {
      std::cout << "; publish " << kLowCmdTopic
                << " raw-no-output-clamp\n";
    } else {
      std::cout << "; no LowCmd publisher will be created\n";
    }

    unitree::robot::ChannelFactory::Instance()->Init(0, args.network);
    auto lowstate_sub =
        std::make_shared<legbot::LowStateSubscriber>(kLowStateTopic);
    if (RealOutputEnabled(args)) {
      lowcmd_pub = std::make_unique<legbot::LowCmdPublisher>(kLowCmdTopic);
    }
    legbot::LegBotDDSStateAdapter state_adapter(args.lowstate_timeout_s);
    state_adapter.SetSubscriber(lowstate_sub);

    if (args.dry_run) {
      RunDryRun(args, state_adapter, &stats, stats_t0);
      exit_reason = g_running ? "dry_run_completed" : "signal";
    } else {
      exit_reason = RunControllerOutput(args, lowstate_sub, lowcmd_pub.get(),
                                        &state_adapter, &stats, stats_t0);
    }

    if (RealOutputEnabled(args) && args.disable_on_exit) {
      exit_command_adapter.DisableBurst(lowcmd_pub.get(), args, &stats,
                                        stats_t0);
    }

    const double elapsed_s = legbot::SecondsSince(stats_t0, legbot::Clock::now());
    legbot::PrintSummary(exit_reason, elapsed_s, stats);
    return 0;
  } catch (const std::exception& e) {
    exit_reason = std::string("exception:") + e.what();
    if (RealOutputEnabled(args) && args.disable_on_exit) {
      try {
        exit_command_adapter.DisableBurst(lowcmd_pub.get(), args, &stats,
                                          stats_t0);
      } catch (...) {
      }
    }
    const double elapsed_s = legbot::SecondsSince(stats_t0, legbot::Clock::now());
    legbot::PrintSummary(exit_reason, elapsed_s, stats);
    std::cerr << "ERROR: " << e.what() << "\n";
    return 1;
  }
}

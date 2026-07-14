#ifndef PROJECT_LEGBOT_SAFETY_SUPERVISOR_H
#define PROJECT_LEGBOT_SAFETY_SUPERVISOR_H

#include "LegBotDDS/LegBotJointMap.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace legbot {

using Clock = std::chrono::steady_clock;

struct BridgeArgs {
  std::string network = "lo";
  std::string stage = "dry-run";
  std::string csv_log_path;
  double duration_s = 8.0;
  double cmd_hz = 100.0;
  double standup_ramp_seconds = 12.0;
  double standup_prehold_seconds = 2.0;
  double standup_kp = 20.0;
  double standup_kd = 3.0;
  double shutdown_ramp_seconds = 8.0;
  double shutdown_prehold_seconds = 1.0;
  double fsm_warmup_seconds = 0.5;
  double mit_output_wait_seconds = 2.0;
  double wait_lowstate_s = 5.0;
  double lowstate_timeout_s = 0.25;
  double max_q_error = 0.45;
  double max_tilt_rad = 0.35;
  double max_gyro_rad_s = 5.0;
  double max_abs_q_error_to_cmd = 0.0;
  double max_abs_qd_des = 0.0;
  double max_abs_tau_ff = 0.0;
  double max_qdes_jump_per_tick = 0.0;
  double max_qdes_jump_hip = 0.0;
  double max_qdes_jump_thigh = 0.0;
  double max_qdes_jump_calf = 0.0;
  double tau_ff_scale = 1.0;
  double locomotion_ramp_seconds = 0.75;
  double locomotion_allstance_seconds = 0.0;
  double csv_hz = 50.0;
  double test_command_ramp_seconds = 2.0;
  double interactive_vx_step = 0.05;
  double interactive_yaw_step = 0.15;
  double interactive_max_vx = 0.25;
  double interactive_max_yaw_rate = 0.8;
  double algo_forward_vx = 0.03;
  double cmd_action_limit = 100.0;
  double cmd_hip_abs_limit = 0.55;
  double cmd_thigh_min = 0.45;
  double cmd_thigh_max = 1.35;
  double cmd_calf_min = -2.75;
  double cmd_calf_max = -1.20;
  double cmd_qdes_delta_limit = 0.03;
  double cmd_qd_delta_limit = 0.5;
  double cmd_tau_limit = 12.0;
  double cmd_kp_min = 0.0;
  double cmd_kp_max = 80.0;
  double cmd_kd_min = 0.0;
  double cmd_kd_max = 6.0;
  double fb_tau_limit = 30.0;
  double fb_qd_limit = 20.0;
  double fb_temp_warning = 70.0;
  double fb_temp_limit = 80.0;
  double fb_hip_abs_limit = 0.70;
  double fb_thigh_min = 0.30;
  double fb_thigh_max = 1.50;
  double fb_calf_min = -2.90;
  double fb_calf_max = -1.00;
  double hard_fault_damping_hold_s = 0.30;
  double test_vx = 0.0;
  double test_vy = 0.0;
  double test_yaw_rate = 0.0;
  double test_gait = 4.0;
  int nudge_joint = 0;
  double nudge_delta = 0.03;
  double nudge_seconds = 1.0;
  int foot_probe_leg = 1;
  std::string foot_probe_axis = "z";
  double foot_probe_delta = 0.01;
  double foot_probe_seconds = 2.0;
  double foot_probe_kp = 80.0;
  double foot_probe_kd = 4.0;
  int single_leg_probe_leg = 1;
  double single_leg_probe_seconds = 2.0;
  bool dry_run = false;
  bool dry_output = false;
  bool real_output_guarded = false;
  bool real_output_raw = false;
  bool test_locomotion = false;
  bool interactive_control = false;
  bool gait0_never_release = false;
  bool joint_nudge_test = false;
  bool foot_delta_probe_test = false;
  bool robot_standing_supported = false;
  bool i_accept_risk = false;
  bool disable_on_exit = false;
  bool allow_long_duration = false;
  bool help = false;
};

struct LoopStats {
  uint64_t loop_count = 0;
  uint64_t publish_count = 0;
  uint64_t lowstate_count = 0;
  double lowstate_interval_sum_s = 0.;
  double lowstate_interval_min_s = std::numeric_limits<double>::infinity();
  double lowstate_interval_max_s = 0.;
  double loop_ms_sum = 0.;
  double loop_ms_max = 0.;
  double max_qerr = 0.;
  double max_tilt = 0.;
  std::vector<double> publish_times_s;
};

inline double SecondsSince(const Clock::time_point& t0,
                           const Clock::time_point& t) {
  return std::chrono::duration<double>(t - t0).count();
}

inline double Clamp(double v, double lo, double hi) {
  return std::min(std::max(v, lo), hi);
}

inline std::array<double, 3> QuatWxyzToRpy(const std::array<float, 4>& quat) {
  double w = quat[0];
  double x = quat[1];
  double y = quat[2];
  double z = quat[3];
  const double norm = std::sqrt(w * w + x * x + y * y + z * z);
  if (norm <= 1.e-9) return {{0., 0., 0.}};
  w /= norm;
  x /= norm;
  y /= norm;
  z /= norm;

  const double sinr_cosp = 2. * (w * x + y * z);
  const double cosr_cosp = 1. - 2. * (x * x + y * y);
  const double roll = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp = 2. * (w * y - z * x);
  const double pitch =
      std::abs(sinp) >= 1. ? std::copysign(M_PI / 2., sinp) : std::asin(sinp);

  const double siny_cosp = 2. * (w * z + x * y);
  const double cosy_cosp = 1. - 2. * (y * y + z * z);
  const double yaw = std::atan2(siny_cosp, cosy_cosp);
  return {{roll, pitch, yaw}};
}

template <typename ArrayT>
std::string ArrayText(const ArrayT& values, int precision) {
  std::ostringstream oss;
  oss << "[" << std::fixed << std::setprecision(precision);
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i) oss << ", ";
    oss << static_cast<double>(values[i]);
  }
  oss << "]";
  return oss.str();
}

inline double MaxAbsError(const std::array<double, kNumJoints>& a,
                          const std::array<double, kNumJoints>& b,
                          int* max_i) {
  double max_err = 0.;
  int idx = 0;
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    const double err = std::abs(a[i] - b[i]);
    if (err > max_err) {
      max_err = err;
      idx = static_cast<int>(i);
    }
  }
  if (max_i) *max_i = idx;
  return max_err;
}

inline void PrintHelp(const char* argv0) {
  std::cout
      << "Usage: " << argv0 << " [options]\n\n"
      << "LegBot DDS bridge first-stage validator. Stage 0 dry-run only reads "
         "rt/lowstate. Stage 1 dry-output runs RobotRunner and prints "
         "SpiCommand summaries, but never publishes rt/lowcmd. Guarded real "
         "output runs the full Cheetah controller and publishes rt/lowcmd "
         "only after safety checks pass.\n\n"
      << "Options:\n"
      << "  --network IFACE                       DDS network interface "
         "(default: lo)\n"
      << "  --stage dry-run|dry-output|real-output-guarded|real-output-raw\n"
      << "                                        Bridge mode "
         "(default: dry-run)\n"
      << "  --dry-run                             Alias for --stage dry-run\n"
      << "  --dry-output                          Alias for --stage dry-output\n"
      << "  --real-output-guarded                 Publish guarded rt/lowcmd\n"
      << "  --real-output-raw                     Publish raw algorithm rt/lowcmd "
         "with feedback safety only; no OutputGuard command clip\n"
      << "  --duration S                          Run duration (default: 8)\n"
      << "  --cmd-hz HZ                           Control loop rate (default: 100)\n"
      << "  --standup-ramp-seconds S              Current q -> stand q ramp "
         "before guarded output (default: 12)\n"
      << "  --standup-prehold-seconds S           Stand hold before guarded "
         "output (default: 2)\n"
      << "  --standup-kp GAIN                     Stand-up q-only kp "
         "(default: 20)\n"
      << "  --standup-kd GAIN                     Stand-up q-only kd "
         "(default: 3)\n"
      << "  --shutdown-ramp-seconds S             Current q -> startup prone q "
         "ramp before disable (default: 8)\n"
      << "  --shutdown-prehold-seconds S          Prone hold before disable "
         "(default: 1)\n"
      << "  --fsm-warmup-seconds S                Request MIT STAND_UP before "
         "LOCOMOTION (default: 0.5)\n"
      << "  --mit-output-wait-seconds S           Run MIT LOCOMOTION while "
         "publishing Q-STAND-HOLD until SpiCommand is valid (default: 2)\n"
      << "  --wait-lowstate-s S                   Initial LowState wait timeout "
         "(default: 5)\n"
      << "  --lowstate-timeout-s S                Runtime LowState timeout "
         "(default: 0.25)\n"
      << "  --max-q-error RAD                     Abort threshold for dry-output "
         "model q target error (default: 0.45)\n"
      << "  --max-tilt-rad RAD                    Abort on max |roll|/|pitch| "
         "(default: 0.35)\n"
      << "  --max-gyro-rad-s RADS                 Abort on large gyro "
         "(default: 5)\n"
      << "  --max-abs-q-error-to-cmd RAD          Guarded q vs q_des limit "
         "(default: 0 disables)\n"
      << "  --max-abs-qd-des RADS                 Guarded qd_des limit "
         "(default: 0 disables)\n"
      << "  --max-abs-tau-ff NM                   Guarded tau_ff limit "
         "(default: 0 disables)\n"
      << "  --max-qdes-jump-per-tick RAD          Guarded q_des jump limit "
         "for all joints; overrides per-joint defaults\n"
      << "  --max-qdes-jump-hip RAD               Hip q_des jump limit "
         "(default: 0 disables)\n"
      << "  --max-qdes-jump-thigh RAD             Thigh q_des jump limit "
         "(default: 0 disables)\n"
      << "  --max-qdes-jump-calf RAD              Calf q_des jump limit "
         "(default: 0 disables)\n"
      << "  --tau-ff-scale SCALE                  Explicit experimental tau_ff "
         "scale before publish (default: 1)\n"
      << "  --locomotion-ramp-seconds S           Key 3 direct output ramp "
         "duration (default: 0.75; 0 disables)\n"
      << "  --locomotion-allstance-seconds S      Key 3 LOCOMOTION all-stance "
         "warmup before requested gait (default: 0 disables)\n"
      << "  --csv-log PATH                        Write MIT direct diagnostic CSV "
         "(default: disabled)\n"
      << "  --csv-hz HZ                           CSV sample rate when enabled "
         "(default: 50)\n"
      << "  --test-locomotion                     After stand-up, force "
         "MIT LOCOMOTION test commands\n"
      << "  --interactive-control                 Keyboard staged real-output "
         "test: 1 q-stand, 2 MIT stand direct, 3 MIT forward direct, "
         "4 down/disable\n"
      << "  --interactive-vx-step CMD             Legacy velocity option; "
         "unused by direct algorithm test (default: 0.05)\n"
      << "  --interactive-yaw-step CMD            Legacy yaw option; unused by "
         "direct algorithm test (default: 0.15)\n"
      << "  --interactive-max-vx CMD              Legacy velocity clamp; unused "
         "by direct algorithm test (default: 0.25)\n"
      << "  --interactive-max-yaw-rate CMD        Legacy yaw clamp; unused by "
         "direct algorithm test (default: 0.8)\n"
      << "  --algo-forward-vx CMD                 Key 3 MIT direct forward vx "
         "(default: 0.03)\n"
      << "  --cmd-action-limit VALUE              OutputGuard action/mode clip "
         "limit (default: 100)\n"
      << "  --cmd-hip-abs-limit RAD               OutputGuard hip |q_des| limit "
         "(default: 0.55)\n"
      << "  --cmd-thigh-min RAD                   OutputGuard thigh min "
         "(default: 0.45)\n"
      << "  --cmd-thigh-max RAD                   OutputGuard thigh max "
         "(default: 1.35)\n"
      << "  --cmd-calf-min RAD                    OutputGuard calf min "
         "(default: -2.75)\n"
      << "  --cmd-calf-max RAD                    OutputGuard calf max "
         "(default: -1.20)\n"
      << "  --cmd-qdes-delta-limit RAD            OutputGuard q_des per-cycle "
         "delta limit (default: 0.03)\n"
      << "  --cmd-qd-delta-limit RADS             OutputGuard qd_des per-cycle "
         "delta limit (default: 0.5)\n"
      << "  --cmd-tau-limit NM                    OutputGuard |tau_ff| limit "
         "(default: 12)\n"
      << "  --cmd-kp-min/--cmd-kp-max             OutputGuard Kp range "
         "(default: 0/80)\n"
      << "  --cmd-kd-min/--cmd-kd-max             OutputGuard Kd range "
         "(default: 0/6)\n"
      << "  --fb-tau-limit NM                     Feedback |tau_est| fault "
         "limit (default: 30)\n"
      << "  --fb-qd-limit RADS                    Feedback |dq| fault limit "
         "(default: 20)\n"
      << "  --fb-temp-warning C                   Feedback temp warning "
         "(default: 70)\n"
      << "  --fb-temp-limit C                     Feedback temp fault limit "
         "(default: 80)\n"
      << "  --fb-hip-abs-limit RAD                Feedback hip |q| limit "
         "(default: 0.70)\n"
      << "  --fb-thigh-min/--fb-thigh-max RAD     Feedback thigh range "
         "(default: 0.30/1.50)\n"
      << "  --fb-calf-min/--fb-calf-max RAD       Feedback calf range "
         "(default: -2.90/-1.00)\n"
      << "  --hard-fault-damping-hold-s S         Current-q damping hold before "
         "disable on hard fault (default: 0.30)\n"
      << "  --test-command-ramp-seconds S         Ramp test vx/vy/yaw input "
         "from zero (default: 2)\n"
      << "  --test-vx CMD                         Locomotion x command "
         "(default: 0)\n"
      << "  --test-vy CMD                         Locomotion y command "
         "(default: 0)\n"
      << "  --test-yaw-rate CMD                   Locomotion yaw command "
         "(default: 0)\n"
      << "  --test-gait ID                        Override cmpc_gait in "
         "locomotion test (default: 4)\n"
      << "  --gait0-never-release                 Keep GAIT0_SHADOW compute-only; "
         "never publish raw gait0\n"
      << "  --joint-nudge-test                    Run q-only single-joint nudge "
         "test, then return to Q-STAND-HOLD\n"
      << "  --nudge-joint DDS_INDEX               DDS joint index for nudge "
         "(default: 0)\n"
      << "  --nudge-delta RAD                     Nudge delta from current q "
         "(default: 0.03)\n"
      << "  --nudge-seconds S                     Nudge hold duration "
         "(default: 1.0)\n"
      << "  --foot-delta-probe-test               Run single-foot Cartesian "
         "delta probe through Cheetah LegController\n"
      << "  --foot-probe-leg LEG                  Model leg index FR=0, FL=1, "
         "RR=2, RL=3 (default: 1)\n"
      << "  --foot-probe-axis x|y|z               Foot probe axis in leg frame "
         "(default: z)\n"
      << "  --foot-probe-delta M                  Foot probe delta in meters "
         "(default: 0.01)\n"
      << "  --foot-probe-seconds S                Foot probe duration "
         "(default: 2.0)\n"
      << "  --foot-probe-kp N_PER_M               Cartesian probe kp "
         "(default: 80)\n"
      << "  --foot-probe-kd N_PER_M_S             Cartesian probe kd "
         "(default: 4)\n"
      << "  --single-leg-probe-leg LEG            Interactive p selected "
         "algorithm leg FR=0, FL=1, RR=2, RL=3 (default: 1)\n"
      << "  --single-leg-probe-seconds S          Interactive p selected-leg "
         "algorithm probe duration (default: 2.0)\n"
      << "  --robot-standing-supported            Required for output modes\n"
      << "  --i-accept-risk                       Required for output modes\n"
      << "  --allow-long-duration                 Allow output duration > 12s\n"
      << "  --disable-on-exit                     Publish disable burst on exit "
         "in real-output modes\n"
      << "  --help                                Show this help\n";
}

inline BridgeArgs ParseArgs(int argc, char** argv) {
  BridgeArgs args;
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    auto need = [&](const char* name) {
      if (i + 1 >= argc) {
        throw std::runtime_error(std::string("missing value for ") + name);
      }
      return std::string(argv[++i]);
    };

    if (key == "--help" || key == "-h") args.help = true;
    else if (key == "--network") args.network = need("--network");
    else if (key == "--stage") args.stage = need("--stage");
    else if (key == "--dry-run") args.stage = "dry-run";
    else if (key == "--dry-output") args.stage = "dry-output";
    else if (key == "--real-output-guarded") args.stage = "real-output-guarded";
    else if (key == "--real-output-raw") args.stage = "real-output-raw";
    else if (key == "--duration") args.duration_s = std::stod(need("--duration"));
    else if (key == "--cmd-hz") args.cmd_hz = std::stod(need("--cmd-hz"));
    else if (key == "--standup-ramp-seconds") args.standup_ramp_seconds = std::stod(need("--standup-ramp-seconds"));
    else if (key == "--standup-prehold-seconds") args.standup_prehold_seconds = std::stod(need("--standup-prehold-seconds"));
    else if (key == "--standup-kp") args.standup_kp = std::stod(need("--standup-kp"));
    else if (key == "--standup-kd") args.standup_kd = std::stod(need("--standup-kd"));
    else if (key == "--shutdown-ramp-seconds") args.shutdown_ramp_seconds = std::stod(need("--shutdown-ramp-seconds"));
    else if (key == "--shutdown-prehold-seconds") args.shutdown_prehold_seconds = std::stod(need("--shutdown-prehold-seconds"));
    else if (key == "--fsm-warmup-seconds") args.fsm_warmup_seconds = std::stod(need("--fsm-warmup-seconds"));
    else if (key == "--mit-output-wait-seconds") args.mit_output_wait_seconds = std::stod(need("--mit-output-wait-seconds"));
    else if (key == "--wait-lowstate-s") args.wait_lowstate_s = std::stod(need("--wait-lowstate-s"));
    else if (key == "--lowstate-timeout-s") args.lowstate_timeout_s = std::stod(need("--lowstate-timeout-s"));
    else if (key == "--max-q-error") args.max_q_error = std::stod(need("--max-q-error"));
    else if (key == "--max-tilt-rad") args.max_tilt_rad = std::stod(need("--max-tilt-rad"));
    else if (key == "--max-gyro-rad-s") args.max_gyro_rad_s = std::stod(need("--max-gyro-rad-s"));
    else if (key == "--max-abs-q-error-to-cmd") args.max_abs_q_error_to_cmd = std::stod(need("--max-abs-q-error-to-cmd"));
    else if (key == "--max-abs-qd-des") args.max_abs_qd_des = std::stod(need("--max-abs-qd-des"));
    else if (key == "--max-abs-tau-ff") args.max_abs_tau_ff = std::stod(need("--max-abs-tau-ff"));
    else if (key == "--max-qdes-jump-per-tick") {
      args.max_qdes_jump_per_tick = std::stod(need("--max-qdes-jump-per-tick"));
      args.max_qdes_jump_hip = args.max_qdes_jump_per_tick;
      args.max_qdes_jump_thigh = args.max_qdes_jump_per_tick;
      args.max_qdes_jump_calf = args.max_qdes_jump_per_tick;
    }
    else if (key == "--max-qdes-jump-hip") args.max_qdes_jump_hip = std::stod(need("--max-qdes-jump-hip"));
    else if (key == "--max-qdes-jump-thigh") args.max_qdes_jump_thigh = std::stod(need("--max-qdes-jump-thigh"));
    else if (key == "--max-qdes-jump-calf") args.max_qdes_jump_calf = std::stod(need("--max-qdes-jump-calf"));
    else if (key == "--tau-ff-scale") args.tau_ff_scale = std::stod(need("--tau-ff-scale"));
    else if (key == "--locomotion-ramp-seconds") args.locomotion_ramp_seconds = std::stod(need("--locomotion-ramp-seconds"));
    else if (key == "--locomotion-allstance-seconds") args.locomotion_allstance_seconds = std::stod(need("--locomotion-allstance-seconds"));
    else if (key == "--csv-log") args.csv_log_path = need("--csv-log");
    else if (key == "--csv-hz") args.csv_hz = std::stod(need("--csv-hz"));
    else if (key == "--test-locomotion") args.test_locomotion = true;
    else if (key == "--interactive-control") args.interactive_control = true;
    else if (key == "--interactive-vx-step") args.interactive_vx_step = std::stod(need("--interactive-vx-step"));
    else if (key == "--interactive-yaw-step") args.interactive_yaw_step = std::stod(need("--interactive-yaw-step"));
    else if (key == "--interactive-max-vx") args.interactive_max_vx = std::stod(need("--interactive-max-vx"));
    else if (key == "--interactive-max-yaw-rate") args.interactive_max_yaw_rate = std::stod(need("--interactive-max-yaw-rate"));
    else if (key == "--algo-forward-vx") args.algo_forward_vx = std::stod(need("--algo-forward-vx"));
    else if (key == "--cmd-action-limit") args.cmd_action_limit = std::stod(need("--cmd-action-limit"));
    else if (key == "--cmd-hip-abs-limit") args.cmd_hip_abs_limit = std::stod(need("--cmd-hip-abs-limit"));
    else if (key == "--cmd-thigh-min") args.cmd_thigh_min = std::stod(need("--cmd-thigh-min"));
    else if (key == "--cmd-thigh-max") args.cmd_thigh_max = std::stod(need("--cmd-thigh-max"));
    else if (key == "--cmd-calf-min") args.cmd_calf_min = std::stod(need("--cmd-calf-min"));
    else if (key == "--cmd-calf-max") args.cmd_calf_max = std::stod(need("--cmd-calf-max"));
    else if (key == "--cmd-qdes-delta-limit") args.cmd_qdes_delta_limit = std::stod(need("--cmd-qdes-delta-limit"));
    else if (key == "--cmd-qd-delta-limit") args.cmd_qd_delta_limit = std::stod(need("--cmd-qd-delta-limit"));
    else if (key == "--cmd-tau-limit") args.cmd_tau_limit = std::stod(need("--cmd-tau-limit"));
    else if (key == "--cmd-kp-min") args.cmd_kp_min = std::stod(need("--cmd-kp-min"));
    else if (key == "--cmd-kp-max") args.cmd_kp_max = std::stod(need("--cmd-kp-max"));
    else if (key == "--cmd-kd-min") args.cmd_kd_min = std::stod(need("--cmd-kd-min"));
    else if (key == "--cmd-kd-max") args.cmd_kd_max = std::stod(need("--cmd-kd-max"));
    else if (key == "--fb-tau-limit") args.fb_tau_limit = std::stod(need("--fb-tau-limit"));
    else if (key == "--fb-qd-limit") args.fb_qd_limit = std::stod(need("--fb-qd-limit"));
    else if (key == "--fb-temp-warning") args.fb_temp_warning = std::stod(need("--fb-temp-warning"));
    else if (key == "--fb-temp-limit") args.fb_temp_limit = std::stod(need("--fb-temp-limit"));
    else if (key == "--fb-hip-abs-limit") args.fb_hip_abs_limit = std::stod(need("--fb-hip-abs-limit"));
    else if (key == "--fb-thigh-min") args.fb_thigh_min = std::stod(need("--fb-thigh-min"));
    else if (key == "--fb-thigh-max") args.fb_thigh_max = std::stod(need("--fb-thigh-max"));
    else if (key == "--fb-calf-min") args.fb_calf_min = std::stod(need("--fb-calf-min"));
    else if (key == "--fb-calf-max") args.fb_calf_max = std::stod(need("--fb-calf-max"));
    else if (key == "--hard-fault-damping-hold-s") args.hard_fault_damping_hold_s = std::stod(need("--hard-fault-damping-hold-s"));
    else if (key == "--test-command-ramp-seconds") args.test_command_ramp_seconds = std::stod(need("--test-command-ramp-seconds"));
    else if (key == "--test-vx") args.test_vx = std::stod(need("--test-vx"));
    else if (key == "--test-vy") args.test_vy = std::stod(need("--test-vy"));
    else if (key == "--test-yaw-rate") args.test_yaw_rate = std::stod(need("--test-yaw-rate"));
    else if (key == "--test-gait") args.test_gait = std::stod(need("--test-gait"));
    else if (key == "--gait0-never-release") args.gait0_never_release = true;
    else if (key == "--joint-nudge-test") args.joint_nudge_test = true;
    else if (key == "--nudge-joint") args.nudge_joint = std::stoi(need("--nudge-joint"));
    else if (key == "--nudge-delta") args.nudge_delta = std::stod(need("--nudge-delta"));
    else if (key == "--nudge-seconds") args.nudge_seconds = std::stod(need("--nudge-seconds"));
    else if (key == "--foot-delta-probe-test") args.foot_delta_probe_test = true;
    else if (key == "--foot-probe-leg") args.foot_probe_leg = std::stoi(need("--foot-probe-leg"));
    else if (key == "--foot-probe-axis") args.foot_probe_axis = need("--foot-probe-axis");
    else if (key == "--foot-probe-delta") args.foot_probe_delta = std::stod(need("--foot-probe-delta"));
    else if (key == "--foot-probe-seconds") args.foot_probe_seconds = std::stod(need("--foot-probe-seconds"));
    else if (key == "--foot-probe-kp") args.foot_probe_kp = std::stod(need("--foot-probe-kp"));
    else if (key == "--foot-probe-kd") args.foot_probe_kd = std::stod(need("--foot-probe-kd"));
    else if (key == "--single-leg-probe-leg") args.single_leg_probe_leg = std::stoi(need("--single-leg-probe-leg"));
    else if (key == "--single-leg-probe-seconds") args.single_leg_probe_seconds = std::stod(need("--single-leg-probe-seconds"));
    else if (key == "--robot-standing-supported") args.robot_standing_supported = true;
    else if (key == "--i-accept-risk") args.i_accept_risk = true;
    else if (key == "--disable-on-exit") args.disable_on_exit = true;
    else if (key == "--allow-long-duration") args.allow_long_duration = true;
    else throw std::runtime_error("unknown option: " + key);
  }
  args.dry_run = args.stage == "dry-run";
  args.dry_output = args.stage == "dry-output";
  args.real_output_guarded = args.stage == "real-output-guarded";
  args.real_output_raw = args.stage == "real-output-raw";
  return args;
}

inline void ValidateArgs(const BridgeArgs& args) {
  if (args.help) return;
  if (!args.dry_run && !args.dry_output && !args.real_output_guarded &&
      !args.real_output_raw) {
    throw std::runtime_error("--stage must be dry-run, dry-output, real-output-guarded or real-output-raw");
  }
  if (args.duration_s < 0.) throw std::runtime_error("--duration must be non-negative");
  if (args.cmd_hz <= 0.) throw std::runtime_error("--cmd-hz must be positive");
  if (args.standup_ramp_seconds < 0. || args.standup_prehold_seconds < 0.) {
    throw std::runtime_error("--standup-ramp-seconds/--standup-prehold-seconds must be non-negative");
  }
  if (args.shutdown_ramp_seconds < 0. || args.shutdown_prehold_seconds < 0.) {
    throw std::runtime_error("--shutdown-ramp-seconds/--shutdown-prehold-seconds must be non-negative");
  }
  if (args.standup_kp < 0. || args.standup_kd < 0.) {
    throw std::runtime_error("--standup-kp/--standup-kd must be non-negative");
  }
  if (args.fsm_warmup_seconds < 0. || args.mit_output_wait_seconds < 0. ||
      args.wait_lowstate_s <= 0. || args.lowstate_timeout_s <= 0.) {
    throw std::runtime_error("--wait-lowstate-s/--lowstate-timeout-s must be positive");
  }
  if (args.max_q_error <= 0. || args.max_tilt_rad <= 0.) {
    throw std::runtime_error("--max-q-error/--max-tilt-rad must be positive");
  }
  if (args.max_gyro_rad_s <= 0. || args.max_abs_q_error_to_cmd < 0. ||
      args.max_abs_qd_des < 0. || args.max_abs_tau_ff < 0. ||
      args.max_qdes_jump_per_tick < 0. || args.max_qdes_jump_hip < 0. ||
      args.max_qdes_jump_thigh < 0. || args.max_qdes_jump_calf < 0. ||
      args.tau_ff_scale < 0. || args.locomotion_ramp_seconds < 0. ||
      args.locomotion_allstance_seconds < 0. || args.csv_hz <= 0. ||
      args.test_command_ramp_seconds < 0. ||
      args.interactive_vx_step < 0. || args.interactive_yaw_step < 0. ||
      args.interactive_max_vx < 0. || args.interactive_max_yaw_rate < 0. ||
      args.cmd_action_limit < 0. || args.cmd_hip_abs_limit <= 0. ||
      args.cmd_qdes_delta_limit < 0. || args.cmd_qd_delta_limit < 0. ||
      args.cmd_tau_limit < 0. || args.cmd_kp_min < 0. ||
      args.cmd_kp_max < args.cmd_kp_min || args.cmd_kd_min < 0. ||
      args.cmd_kd_max < args.cmd_kd_min || args.fb_tau_limit < 0. ||
      args.fb_qd_limit < 0. || args.fb_temp_warning < 0. ||
      args.fb_temp_limit < args.fb_temp_warning ||
      args.fb_hip_abs_limit <= 0. ||
      args.hard_fault_damping_hold_s < 0. ||
      args.nudge_seconds < 0. || args.foot_probe_seconds < 0. ||
      args.foot_probe_kp < 0. || args.foot_probe_kd < 0. ||
      args.single_leg_probe_seconds < 0.) {
    throw std::runtime_error("guarded thresholds and scales must be non-negative");
  }
  if (args.nudge_joint < 0 ||
      args.nudge_joint >= static_cast<int>(kNumJoints)) {
    throw std::runtime_error("--nudge-joint must be in [0, 11]");
  }
  if (args.foot_probe_leg < 0 ||
      args.foot_probe_leg >= static_cast<int>(kNumLegs)) {
    throw std::runtime_error("--foot-probe-leg must be in [0, 3]");
  }
  if (args.single_leg_probe_leg < 0 ||
      args.single_leg_probe_leg >= static_cast<int>(kNumLegs)) {
    throw std::runtime_error("--single-leg-probe-leg must be in [0, 3]");
  }
  if (args.foot_probe_axis != "x" && args.foot_probe_axis != "y" &&
      args.foot_probe_axis != "z") {
    throw std::runtime_error("--foot-probe-axis must be x, y, or z");
  }
  if (args.cmd_thigh_min > args.cmd_thigh_max ||
      args.cmd_calf_min > args.cmd_calf_max ||
      args.fb_thigh_min > args.fb_thigh_max ||
      args.fb_calf_min > args.fb_calf_max) {
    throw std::runtime_error("joint min limit must be <= max limit");
  }
  if (args.dry_output || args.real_output_guarded || args.real_output_raw) {
    if (!args.robot_standing_supported) {
      throw std::runtime_error("refusing output mode without --robot-standing-supported");
    }
    if (!args.i_accept_risk) {
      throw std::runtime_error("refusing output mode without --i-accept-risk");
    }
    if (args.duration_s > 12. && !args.allow_long_duration) {
      throw std::runtime_error("refusing output mode --duration > 12 without --allow-long-duration");
    }
  }
}

inline void RecordLowStateTiming(uint32_t tick, LoopStats* stats,
                                 Clock::time_point* last_sample_t,
                                 uint32_t* last_tick) {
  if (stats->lowstate_count > 0 && tick == *last_tick) return;
  const auto now = Clock::now();
  if (stats->lowstate_count > 0) {
    const double dt = SecondsSince(*last_sample_t, now);
    stats->lowstate_interval_sum_s += dt;
    stats->lowstate_interval_min_s =
        std::min(stats->lowstate_interval_min_s, dt);
    stats->lowstate_interval_max_s =
        std::max(stats->lowstate_interval_max_s, dt);
  }
  *last_sample_t = now;
  *last_tick = tick;
  ++stats->lowstate_count;
}

inline void PrintSummary(const std::string& exit_reason, double elapsed_s,
                         const LoopStats& stats) {
  const double lowstate_mean =
      stats.lowstate_count >= 2
          ? stats.lowstate_interval_sum_s /
                static_cast<double>(stats.lowstate_count - 1)
          : std::numeric_limits<double>::quiet_NaN();
  const double loop_ms_mean =
      stats.loop_count > 0
          ? stats.loop_ms_sum / static_cast<double>(stats.loop_count)
          : std::numeric_limits<double>::quiet_NaN();
  std::cout << std::fixed << std::setprecision(6)
            << "exit_reason=" << exit_reason << "\n"
            << "elapsed_s=" << elapsed_s << "\n"
            << "loop_count=" << stats.loop_count << "\n"
            << "publish_count=" << stats.publish_count << "\n"
            << "lowstate_count=" << stats.lowstate_count << "\n"
            << "lowstate_interval_mean_s=" << lowstate_mean << "\n"
            << "lowstate_interval_min_s="
            << (stats.lowstate_count >= 2
                    ? stats.lowstate_interval_min_s
                    : std::numeric_limits<double>::quiet_NaN())
            << "\n"
            << "lowstate_interval_max_s="
            << (stats.lowstate_count >= 2
                    ? stats.lowstate_interval_max_s
                    : std::numeric_limits<double>::quiet_NaN())
            << "\n"
            << "loop_ms_mean=" << loop_ms_mean << "\n"
            << "loop_ms_max=" << stats.loop_ms_max << "\n"
            << "max_qerr=" << stats.max_qerr << "\n"
            << "max_tilt=" << stats.max_tilt << "\n";
}

}  // namespace legbot

#endif  // PROJECT_LEGBOT_SAFETY_SUPERVISOR_H

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

#include <Configuration.h>
#include <Dynamics/LegBot.h>
#include <Controllers/LegController.h>
#include <SimUtilities/SimulatorMessage.h>
#include <SimUtilities/SpineBoard.h>
#include <Utilities/SharedMemory.h>
#include <Math/orientation_tools.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr const char* kDefaultXml =
    "models/MJCF/legbot/legbot_mpc_scene_project.xml";
constexpr std::array<const char*, 4> kLegs = {"FR", "FL", "RR", "RL"};
constexpr std::array<const char*, 3> kJointSuffix = {"hip", "thigh", "calf"};

struct Args {
  std::string xml = std::string(THIS_COM) + kDefaultXml;
  double duration = 10.0;
  double speed = 0.4;
  double height = 0.2775;
  double control_dt = 0.001;
  double locomotion_start = 2.0;
  double trot_start = -1.0;
  double speed_ramp = 2.0;
  bool realtime = false;
  bool viewer = false;
  bool debug_control = false;
  bool debug_height = false;
  int render_decimation = 20;
  int viewer_width = 960;
  int viewer_height = 720;
};

struct JointMap {
  std::array<int, 12> qpos{};
  std::array<int, 12> qvel{};
  std::array<int, 12> ctrl{};
  std::array<int, 4> foot_geom{};
  int base_qpos = -1;
  int base_qvel = -1;
  int base_body = -1;
  int ground_geom = -1;
};

void usage(const char* argv0) {
  std::printf(
      "Usage: %s [--xml path] [--duration sec] [--speed mps] "
      "[--height m] [--control-dt sec] [--locomotion-start sec] "
      "[--trot-start sec] [--speed-ramp sec] "
      "[--viewer] [--realtime] [--render-decimation n] "
      "[--viewer-width px] [--viewer-height px] "
      "[--debug-control] [--debug-height]\n",
      argv0);
}

Args parseArgs(int argc, char** argv) {
  Args args;
  for (int i = 1; i < argc; i++) {
    std::string key(argv[i]);
    auto needValue = [&](const char* name) -> const char* {
      if (i + 1 >= argc) {
        usage(argv[0]);
        throw std::runtime_error(std::string("missing value for ") + name);
      }
      return argv[++i];
    };
    if (key == "--xml") {
      args.xml = needValue("--xml");
    } else if (key == "--duration") {
      args.duration = std::atof(needValue("--duration"));
    } else if (key == "--speed") {
      args.speed = std::atof(needValue("--speed"));
    } else if (key == "--height") {
      args.height = std::atof(needValue("--height"));
    } else if (key == "--control-dt") {
      args.control_dt = std::atof(needValue("--control-dt"));
    } else if (key == "--locomotion-start") {
      args.locomotion_start = std::atof(needValue("--locomotion-start"));
    } else if (key == "--trot-start") {
      args.trot_start = std::atof(needValue("--trot-start"));
    } else if (key == "--speed-ramp") {
      args.speed_ramp = std::atof(needValue("--speed-ramp"));
    } else if (key == "--viewer") {
      args.viewer = true;
    } else if (key == "--realtime") {
      args.realtime = true;
    } else if (key == "--debug-control") {
      args.debug_control = true;
    } else if (key == "--debug-height") {
      args.debug_height = true;
    } else if (key == "--render-decimation") {
      args.render_decimation = std::atoi(needValue("--render-decimation"));
      if (args.render_decimation < 1) args.render_decimation = 1;
    } else if (key == "--viewer-width") {
      args.viewer_width = std::atoi(needValue("--viewer-width"));
      if (args.viewer_width < 1) args.viewer_width = 960;
    } else if (key == "--viewer-height") {
      args.viewer_height = std::atoi(needValue("--viewer-height"));
      if (args.viewer_height < 1) args.viewer_height = 720;
    } else if (key == "--help" || key == "-h") {
      usage(argv[0]);
      std::exit(0);
    } else {
      usage(argv[0]);
      throw std::runtime_error("unknown argument: " + key);
    }
  }
  return args;
}

double speedCommandAtTime(const Args& args, double time) {
  const double ramp_start =
      args.trot_start >= 0.0 ? args.trot_start : args.locomotion_start;
  if (time < ramp_start) return 0.0;
  if (args.speed_ramp <= 1e-6) return args.speed;
  double alpha = (time - ramp_start) / args.speed_ramp;
  alpha = std::max(0.0, std::min(1.0, alpha));
  return args.speed * alpha;
}

int requireId(const mjModel* model, int type, const std::string& name) {
  int id = mj_name2id(model, type, name.c_str());
  if (id < 0) throw std::runtime_error("MuJoCo object not found: " + name);
  return id;
}

JointMap buildJointMap(const mjModel* model) {
  JointMap map;
  int freejoint = requireId(model, mjOBJ_JOINT, "joint_fixed_world");
  map.base_qpos = model->jnt_qposadr[freejoint];
  map.base_qvel = model->jnt_dofadr[freejoint];
  map.base_body = requireId(model, mjOBJ_BODY, "base");

  for (int leg = 0; leg < 4; leg++) {
    map.foot_geom[leg] = requireId(model, mjOBJ_GEOM, kLegs[leg]);
    for (int joint = 0; joint < 3; joint++) {
      int idx = 3 * leg + joint;
      std::string name = std::string(kLegs[leg]) + "_" + kJointSuffix[joint] +
                         "_joint";
      int jid = requireId(model, mjOBJ_JOINT, name);
      int aid = requireId(model, mjOBJ_ACTUATOR, name);
      map.qpos[idx] = model->jnt_qposadr[jid];
      map.qvel[idx] = model->jnt_dofadr[jid];
      map.ctrl[idx] = aid;
    }
  }
  map.ground_geom = requireId(model, mjOBJ_GEOM, "ground");
  return map;
}

void printGeomMap(const mjModel* model, const JointMap& map) {
  std::printf("[GEOM_MAP] foot geom ids: FR=%d FL=%d RR=%d RL=%d ground=%d\n",
              map.foot_geom[0], map.foot_geom[1], map.foot_geom[2],
              map.foot_geom[3], map.ground_geom);
  for (int i = 0; i < model->ngeom; ++i) {
    const char* name = mj_id2name(model, mjOBJ_GEOM, i);
    if (name) {
      std::printf("[GEOM] id=%d name=%s body=%d\n", i, name,
                  model->geom_bodyid[i]);
    }
  }
}

std::array<double, 3> nominalLegQ(const char* leg, double height) {
  const double l1 = 0.1985;
  const double l2 = 0.214;
  const bool right = std::strcmp(leg, "FR") == 0 || std::strcmp(leg, "RR") == 0;
  const double side = right ? -1.0 : 1.0;
  double c2 = (height * height - l1 * l1 - l2 * l2) / (2.0 * l1 * l2);
  c2 = std::max(-0.98, std::min(0.98, c2));
  double q_knee = -std::acos(c2);
  double q_thigh = -std::atan2(l2 * std::sin(q_knee), l1 + l2 * std::cos(q_knee));
  return {0.04 * side, q_thigh, q_knee};
}

void resetRobot(const mjModel* model, const JointMap& map, mjData* data,
                double height) {
  mju_zero(data->qpos, model->nq);
  mju_zero(data->qvel, model->nv);
  data->qpos[map.base_qpos + 0] = 0.0;
  data->qpos[map.base_qpos + 1] = 0.0;
  data->qpos[map.base_qpos + 2] = height + 0.025;
  data->qpos[map.base_qpos + 3] = 1.0;
  data->qpos[map.base_qpos + 4] = 0.0;
  data->qpos[map.base_qpos + 5] = 0.0;
  data->qpos[map.base_qpos + 6] = 0.0;

  for (int leg = 0; leg < 4; leg++) {
    auto q = nominalLegQ(kLegs[leg], height);
    for (int joint = 0; joint < 3; joint++) {
      data->qpos[map.qpos[3 * leg + joint]] = q[joint];
    }
  }
}

Eigen::Quaterniond mujocoQuat(const mjData* data, const JointMap& map) {
  return Eigen::Quaterniond(data->qpos[map.base_qpos + 3],
                            data->qpos[map.base_qpos + 4],
                            data->qpos[map.base_qpos + 5],
                            data->qpos[map.base_qpos + 6])
      .normalized();
}

void fillRobotInputs(const mjModel* model, const mjData* data,
                     const JointMap& map, SimulatorSyncronizedMessage& message,
                     double speed) {
  (void)model;
  auto& simToRobot = message.simToRobot;
  simToRobot.robotType = RobotType::LEGBOT;
  simToRobot.mode = SimulatorMode::RUN_CONTROLLER;

  simToRobot.gamepadCommand.zero();
  simToRobot.gamepadCommand.leftStickAnalog[1] = static_cast<float>(speed);
  simToRobot.gamepadCommand.rightStickAnalog[0] = 0.f;

  for (int leg = 0; leg < 4; leg++) {
    simToRobot.spiData.q_abad[leg] = data->qpos[map.qpos[3 * leg + 0]];
    simToRobot.spiData.q_hip[leg] = data->qpos[map.qpos[3 * leg + 1]];
    simToRobot.spiData.q_knee[leg] = data->qpos[map.qpos[3 * leg + 2]];
    simToRobot.spiData.qd_abad[leg] = data->qvel[map.qvel[3 * leg + 0]];
    simToRobot.spiData.qd_hip[leg] = data->qvel[map.qvel[3 * leg + 1]];
    simToRobot.spiData.qd_knee[leg] = data->qvel[map.qvel[3 * leg + 2]];
    simToRobot.spiData.flags[leg] = 0;
  }
  simToRobot.spiData.spi_driver_status = 0;

  Eigen::Quaterniond q = mujocoQuat(data, map);
  Eigen::Matrix3d R = q.toRotationMatrix();
  Eigen::Vector3d v_world(data->qvel[map.base_qvel + 0],
                          data->qvel[map.base_qvel + 1],
                          data->qvel[map.base_qvel + 2]);
  Eigen::Vector3d w_world(data->qvel[map.base_qvel + 3],
                          data->qvel[map.base_qvel + 4],
                          data->qvel[map.base_qvel + 5]);
  Eigen::Vector3d v_body = R.transpose() * v_world;
  Eigen::Vector3d w_body = R.transpose() * w_world;

  simToRobot.cheaterState.orientation =
      Quat<double>(q.w(), q.x(), q.y(), q.z());
  simToRobot.cheaterState.position =
      Vec3<double>(data->qpos[map.base_qpos + 0],
                   data->qpos[map.base_qpos + 1],
                   data->qpos[map.base_qpos + 2]);
  simToRobot.cheaterState.vBody = v_body;
  simToRobot.cheaterState.omegaBody = w_body;
  simToRobot.cheaterState.acceleration = Vec3<double>::Zero();

  simToRobot.vectorNav.quat =
      Quat<float>(static_cast<float>(q.x()), static_cast<float>(q.y()),
                  static_cast<float>(q.z()), static_cast<float>(q.w()));
  simToRobot.vectorNav.gyro = w_body.cast<float>();
  simToRobot.vectorNav.accelerometer = Vec3<float>::Zero();
}

void applyControllerOutput(const JointMap& map, const SpiData& spiData,
                           std::array<SpineBoard, 4>& spineBoards,
                           mjData* data) {
  for (int leg = 0; leg < 4; leg++) {
    spineBoards[leg].run();
    for (int joint = 0; joint < 3; joint++) {
      int idx = 3 * leg + joint;
      (void)spiData;
      data->ctrl[map.ctrl[idx]] = spineBoards[leg].torque_out[joint];
    }
  }
}

void printContactForceDebug(const mjModel* model, const mjData* data,
                            const JointMap& map) {
  double footFn[4] = {0.0, 0.0, 0.0, 0.0};
  int footContactCount[4] = {0, 0, 0, 0};
  int nonFootContactCount = 0;

  for (int ci = 0; ci < data->ncon; ++ci) {
    const mjContact& contact = data->contact[ci];
    const int g1 = contact.geom1;
    const int g2 = contact.geom2;
    bool matchedFoot = false;

    for (int leg = 0; leg < 4; ++leg) {
      if (g1 == map.foot_geom[leg] || g2 == map.foot_geom[leg]) {
        mjtNum force6[6] = {0, 0, 0, 0, 0, 0};
        mj_contactForce(model, data, ci, force6);
        footFn[leg] += static_cast<double>(force6[0]);
        footContactCount[leg] += 1;
        matchedFoot = true;
      }
    }

    if (!matchedFoot) ++nonFootContactCount;
  }

  const double sumFn = footFn[0] + footFn[1] + footFn[2] + footFn[3];
  std::printf(
      "[CONTACT_FORCE_DEBUG] Fn=[%.3f %.3f %.3f %.3f] sum_Fn=%.3f "
      "contacts=[%d %d %d %d] ncon=%d nonfoot=%d body_z=%.6f "
      "base_body_com_z=%.6f qvel_z=%.6f\n",
      footFn[0], footFn[1], footFn[2], footFn[3], sumFn,
      footContactCount[0], footContactCount[1], footContactCount[2],
      footContactCount[3], data->ncon, nonFootContactCount,
      data->qpos[map.base_qpos + 2], data->xipos[3 * map.base_body + 2],
      data->qvel[map.base_qvel + 2]);
}

double sumFootNormalForce(const mjModel* model, const mjData* data,
                          const JointMap& map) {
  double sumFn = 0.0;
  for (int ci = 0; ci < data->ncon; ++ci) {
    const mjContact& contact = data->contact[ci];
    for (int leg = 0; leg < 4; ++leg) {
      if (contact.geom1 == map.foot_geom[leg] ||
          contact.geom2 == map.foot_geom[leg]) {
        mjtNum force6[6] = {0, 0, 0, 0, 0, 0};
        mj_contactForce(model, data, ci, force6);
        sumFn += static_cast<double>(force6[0]);
        break;
      }
    }
  }
  return sumFn;
}

void printVHeightDebug(const mjModel* model, double time, double speed_cmd,
                       const mjData* data, const JointMap& map,
                       const SpiCommand& cmd) {
  static Quadruped<double> legbot = buildLegBot<double>();
  double deltaPFootZSum = 0.0;
  double qerrSquared = 0.0;
  double tauFFSquared = 0.0;
  double tauPDSquared = 0.0;
  double tauTotalSquared = 0.0;
  double ctrlSquared = 0.0;

  for (int leg = 0; leg < 4; ++leg) {
    const int i = 3 * leg;
    const double jointQ[3] = {
        data->qpos[map.qpos[i + 0]],
        data->qpos[map.qpos[i + 1]],
        data->qpos[map.qpos[i + 2]],
    };
    const double qd[3] = {
        data->qvel[map.qvel[i + 0]],
        data->qvel[map.qvel[i + 1]],
        data->qvel[map.qvel[i + 2]],
    };
    const double qDes[3] = {
        cmd.q_des_abad[leg],
        cmd.q_des_hip[leg],
        cmd.q_des_knee[leg],
    };
    const double qdDes[3] = {
        cmd.qd_des_abad[leg],
        cmd.qd_des_hip[leg],
        cmd.qd_des_knee[leg],
    };
    const double kp[3] = {
        cmd.kp_abad[leg],
        cmd.kp_hip[leg],
        cmd.kp_knee[leg],
    };
    const double kd[3] = {
        cmd.kd_abad[leg],
        cmd.kd_hip[leg],
        cmd.kd_knee[leg],
    };
    const double tauFF[3] = {
        cmd.tau_abad_ff[leg],
        cmd.tau_hip_ff[leg],
        cmd.tau_knee_ff[leg],
    };
    Vec3<double> qVec(jointQ[0], jointQ[1], jointQ[2]);
    Vec3<double> qDesVec(qDes[0], qDes[1], qDes[2]);
    Vec3<double> pFoot = Vec3<double>::Zero();
    Vec3<double> pFootDes = Vec3<double>::Zero();
    computeLegJacobianAndPosition(legbot, qVec,
                                  static_cast<Mat3<double>*>(nullptr),
                                  &pFoot, leg);
    computeLegJacobianAndPosition(legbot, qDesVec,
                                  static_cast<Mat3<double>*>(nullptr),
                                  &pFootDes, leg);
    deltaPFootZSum += pFootDes[2] - pFoot[2];

    for (int joint = 0; joint < 3; ++joint) {
      const double qerr = qDes[joint] - jointQ[joint];
      const double qderr = qdDes[joint] - qd[joint];
      const double tauPD = kp[joint] * qerr + kd[joint] * qderr;
      const double tauTotal = tauFF[joint] + tauPD;
      qerrSquared += qerr * qerr;
      tauFFSquared += tauFF[joint] * tauFF[joint];
      tauPDSquared += tauPD * tauPD;
      tauTotalSquared += tauTotal * tauTotal;
      ctrlSquared += data->ctrl[map.ctrl[i + joint]] *
                     data->ctrl[map.ctrl[i + joint]];
    }
  }

  const double sumFn = sumFootNormalForce(model, data, map);
  std::printf(
      "[VHEIGHT] mode=sim t=%.3f body_z=%.6f target_z=nan "
      "pos_err_z=nan vel_z=%.6f op_cmd_z=nan Kp_z=nan Kd_z=nan "
      "mpc_mass=14.000000 required_Fz=nan wbc_sum_Fr_z=nan "
      "mujoco_sum_Fn=%.3f deltaPFoot_z_avg=%.6f qerr_norm=%.6f "
      "tau_ff_norm=%.6f tau_pd_norm=%.6f tau_total_norm=%.6f "
      "ctrl_norm=%.6f qvel_z=%.6f speed_cmd=%.3f\n",
      time, data->qpos[map.base_qpos + 2], data->qvel[map.base_qvel + 2],
      sumFn, deltaPFootZSum / 4.0, std::sqrt(qerrSquared),
      std::sqrt(tauFFSquared), std::sqrt(tauPDSquared),
      std::sqrt(tauTotalSquared), std::sqrt(ctrlSquared),
      data->qvel[map.base_qvel + 2], speed_cmd);
}

void printControlDebug(const mjModel* model, double time, double speed_cmd,
                       const mjData* data, const JointMap& map,
                       const SpiCommand& cmd,
                       const std::array<SpineBoard, 4>& spineBoards) {
  static Quadruped<double> legbot = buildLegBot<double>();
  Eigen::Quaterniond q = mujocoQuat(data, map);
  Vec3<double> rpy = ori::quatToRPY(Quat<double>(q.w(), q.x(), q.y(), q.z()));
  Eigen::Matrix3d R = q.toRotationMatrix();
  Eigen::Vector3d v_world(data->qvel[map.base_qvel + 0],
                          data->qvel[map.base_qvel + 1],
                          data->qvel[map.base_qvel + 2]);
  Eigen::Vector3d w_world(data->qvel[map.base_qvel + 3],
                          data->qvel[map.base_qvel + 4],
                          data->qvel[map.base_qvel + 5]);
  Eigen::Vector3d v_body = R.transpose() * v_world;
  Eigen::Vector3d w_body = R.transpose() * w_world;
  std::printf(
      "[MuJoCo LegBot debug] t=%.3f speed_cmd=%.3f "
      "pos=[%.3f %.3f %.3f] rpy=[%.3f %.3f %.3f] "
      "vBody=[%.3f %.3f %.3f] wBody=[%.3f %.3f %.3f] "
      "flags=[%d %d %d %d]\n",
      time, speed_cmd, data->qpos[map.base_qpos + 0],
      data->qpos[map.base_qpos + 1], data->qpos[map.base_qpos + 2],
      rpy[0], rpy[1], rpy[2], v_body[0], v_body[1], v_body[2],
      w_body[0], w_body[1], w_body[2], cmd.flags[0], cmd.flags[1],
      cmd.flags[2], cmd.flags[3]);
  for (int leg = 0; leg < 4; leg++) {
    const int i = 3 * leg;
    const double jointQ[3] = {
        data->qpos[map.qpos[i + 0]],
        data->qpos[map.qpos[i + 1]],
        data->qpos[map.qpos[i + 2]],
    };
    const double qd[3] = {
        data->qvel[map.qvel[i + 0]],
        data->qvel[map.qvel[i + 1]],
        data->qvel[map.qvel[i + 2]],
    };
    const double qDes[3] = {
        cmd.q_des_abad[leg],
        cmd.q_des_hip[leg],
        cmd.q_des_knee[leg],
    };
    const double qdDes[3] = {
        cmd.qd_des_abad[leg],
        cmd.qd_des_hip[leg],
        cmd.qd_des_knee[leg],
    };
    const double kp[3] = {
        cmd.kp_abad[leg],
        cmd.kp_hip[leg],
        cmd.kp_knee[leg],
    };
    const double kd[3] = {
        cmd.kd_abad[leg],
        cmd.kd_hip[leg],
        cmd.kd_knee[leg],
    };
    const double tauFF[3] = {
        cmd.tau_abad_ff[leg],
        cmd.tau_hip_ff[leg],
        cmd.tau_knee_ff[leg],
    };
    const double tauPD[3] = {
        kp[0] * (qDes[0] - jointQ[0]) + kd[0] * (qdDes[0] - qd[0]),
        kp[1] * (qDes[1] - jointQ[1]) + kd[1] * (qdDes[1] - qd[1]),
        kp[2] * (qDes[2] - jointQ[2]) + kd[2] * (qdDes[2] - qd[2]),
    };
    const double tauBeforeClip[3] = {
        tauFF[0] + tauPD[0],
        tauFF[1] + tauPD[1],
        tauFF[2] + tauPD[2],
    };
    Vec3<double> qVec(jointQ[0], jointQ[1], jointQ[2]);
    Vec3<double> qDesVec(qDes[0], qDes[1], qDes[2]);
    Vec3<double> pFoot = Vec3<double>::Zero();
    Vec3<double> pFootDes = Vec3<double>::Zero();
    computeLegJacobianAndPosition(legbot, qVec,
                                  static_cast<Mat3<double>*>(nullptr),
                                  &pFoot, leg);
    computeLegJacobianAndPosition(legbot, qDesVec,
                                  static_cast<Mat3<double>*>(nullptr),
                                  &pFootDes, leg);
    Vec3<double> deltaPFoot = pFootDes - pFoot;
    std::printf(
        "  %s q=[% .3f % .3f % .3f] qDes=[% .3f % .3f % .3f] "
        "qd=[% .3f % .3f % .3f] qdDes=[% .3f % .3f % .3f]\n",
        kLegs[leg], jointQ[0], jointQ[1], jointQ[2], qDes[0], qDes[1], qDes[2],
        qd[0], qd[1], qd[2], qdDes[0], qdDes[1], qdDes[2]);
    std::printf(
        "     kp=[% .1f % .1f % .1f] kd=[% .1f % .1f % .1f] "
        "tauFF=[% .2f % .2f % .2f] tauPD=[% .2f % .2f % .2f] "
        "tauBefore=[% .2f % .2f % .2f] tauAfter=[% .2f % .2f % .2f] "
        "ctrl=[% .2f % .2f % .2f]\n",
        kp[0], kp[1], kp[2], kd[0], kd[1], kd[2], tauFF[0], tauFF[1],
        tauFF[2], tauPD[0], tauPD[1], tauPD[2], tauBeforeClip[0],
        tauBeforeClip[1], tauBeforeClip[2],
        spineBoards[leg].torque_out[0], spineBoards[leg].torque_out[1],
        spineBoards[leg].torque_out[2], data->ctrl[map.ctrl[i + 0]],
        data->ctrl[map.ctrl[i + 1]], data->ctrl[map.ctrl[i + 2]]);
    std::printf(
        "     pFoot(q)=[% .4f % .4f % .4f] "
        "pFoot(qDes)=[% .4f % .4f % .4f] "
        "deltaPFoot=[% .4f % .4f % .4f]\n",
        pFoot[0], pFoot[1], pFoot[2], pFootDes[0], pFootDes[1],
        pFootDes[2], deltaPFoot[0], deltaPFoot[1], deltaPFoot[2]);
  }
  printVHeightDebug(model, time, speed_cmd, data, map, cmd);
  printContactForceDebug(model, data, map);
}

void printHeightDebug(double time, double speed_cmd, const mjData* data,
                      const JointMap& map) {
  std::printf(
      "[HEIGHT_DEBUG MuJoCo] t=%.3f speed_cmd=%.3f "
      "base_qpos_z=%.6f base_body_com_z=%.6f base_xpos_z=%.6f\n",
      time, speed_cmd, data->qpos[map.base_qpos + 2],
      data->xipos[3 * map.base_body + 2],
      data->xpos[3 * map.base_body + 2]);
}

void requestControlParameter(SimulatorSyncronizedMessage& message,
                             ControlParameterRequestKind kind,
                             const char* name, double value) {
  auto& request = message.simToRobot.controlParameterRequest;
  auto& response = message.robotToSim.controlParameterResponse;
  request.requestNumber = response.requestNumber + 1;
  request.requestKind = kind;
  request.parameterKind = ControlParameterValueKind::DOUBLE;
  request.value.d = value;
  std::snprintf(request.name, sizeof(request.name), "%s", name);
  message.simToRobot.mode = SimulatorMode::RUN_CONTROL_PARAMETERS;
  message.simulatorIsDone();
  message.waitForRobot();
}

void requestControlMode(SimulatorSyncronizedMessage& message, int mode) {
  requestControlParameter(message,
                          ControlParameterRequestKind::SET_ROBOT_PARAM_BY_NAME,
                          "control_mode", static_cast<double>(mode));
}

void requestUserParameter(SimulatorSyncronizedMessage& message,
                          const char* name, double value) {
  requestControlParameter(message,
                          ControlParameterRequestKind::SET_USER_PARAM_BY_NAME,
                          name, value);
}

class MujocoViewer {
 public:
  void init(const mjModel* model, int width, int height) {
    if (!glfwInit()) throw std::runtime_error("glfwInit failed");
    window_ = glfwCreateWindow(width, height, "LegBot MuJoCo MPC/WBC Bridge", nullptr,
                               nullptr);
    if (!window_) throw std::runtime_error("glfwCreateWindow failed");
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(0);

    mjv_defaultCamera(&camera_);
    mjv_defaultOption(&option_);
    mjv_defaultScene(&scene_);
    mjr_defaultContext(&context_);
    camera_.type = mjCAMERA_FREE;
    camera_.distance = 2.2;
    camera_.azimuth = 135;
    camera_.elevation = -20;
    mjv_makeScene(model, &scene_, 2000);
    mjr_makeContext(model, &context_, mjFONTSCALE_150);
  }

  bool alive() const { return window_ && !glfwWindowShouldClose(window_); }

  void render(const mjModel* model, mjData* data, const JointMap& map) {
    if (!window_) return;
    camera_.lookat[0] = data->qpos[map.base_qpos + 0];
    camera_.lookat[1] = data->qpos[map.base_qpos + 1];
    camera_.lookat[2] = 0.25;
    int width = 0;
    int height = 0;
    glfwGetFramebufferSize(window_, &width, &height);
    mjrRect viewport = {0, 0, width, height};
    mjv_updateScene(model, data, &option_, nullptr, &camera_, mjCAT_ALL,
                    &scene_);
    mjr_render(viewport, &scene_, &context_);
    glfwSwapBuffers(window_);
    glfwPollEvents();
  }

  ~MujocoViewer() {
    if (window_) {
      mjr_freeContext(&context_);
      mjv_freeScene(&scene_);
      glfwDestroyWindow(window_);
      glfwTerminate();
    }
  }

 private:
  GLFWwindow* window_ = nullptr;
  mjvCamera camera_;
  mjvOption option_;
  mjvScene scene_;
  mjrContext context_;
};

}  // namespace

int main(int argc, char** argv) {
  try {
    Args args = parseArgs(argc, argv);

    char error[1024] = {0};
    mjModel* model = mj_loadXML(args.xml.c_str(), nullptr, error, sizeof(error));
    if (!model) throw std::runtime_error(std::string("mj_loadXML failed: ") + error);
    mjData* data = mj_makeData(model);
    JointMap map = buildJointMap(model);
    resetRobot(model, map, data, args.height);
    mj_forward(model, data);
    MujocoViewer viewer;
    if (args.viewer) viewer.init(model, args.viewer_width, args.viewer_height);

    SharedMemoryObject<SimulatorSyncronizedMessage> sharedMemory;
    sharedMemory.createNew(DEVELOPMENT_SIMULATOR_SHARED_MEMORY_NAME, true);
    sharedMemory().init();
    sharedMemory().simToRobot.robotType = RobotType::LEGBOT;
    sharedMemory().robotToSim.robotType = RobotType::LEGBOT;

    std::array<SpineBoard, 4> spineBoards;
    for (int leg = 0; leg < 4; leg++) {
      spineBoards[leg].init(Quadruped<float>::getSideSign(leg), leg);
      spineBoards[leg].data = &sharedMemory().simToRobot.spiData;
      spineBoards[leg].cmd = &sharedMemory().robotToSim.spiCommand;
      spineBoards[leg].resetData();
      spineBoards[leg].resetCommand();
    }

	    std::printf("[MuJoCo LegBot] XML: %s\n", args.xml.c_str());
	    std::printf("[MuJoCo LegBot] Start mit_ctrl in another shell: "
	                "./build-legbot-check/user/MIT_Controller/mit_ctrl v s\n");
    if (args.debug_control) {
      printGeomMap(model, map);
    }

    auto wallStart = std::chrono::steady_clock::now();
    if (args.trot_start < 0.0) args.trot_start = args.locomotion_start + 1.5;
    double nextControl = 0.0;
    bool locomotionRequested = false;
    bool trotRequested = false;
    double nextDebugPrint = 0.0;
    long long stepCount = 0;
    while (data->time < args.duration && (!args.viewer || viewer.alive())) {
      if (!locomotionRequested && data->time >= args.locomotion_start) {
        std::printf("[MuJoCo LegBot] Requesting LOCOMOTION at t=%.3f\n",
                    data->time);
        requestControlMode(sharedMemory(), 4);
        locomotionRequested = true;
      }
      if (locomotionRequested && !trotRequested && data->time >= args.trot_start) {
        std::printf("[MuJoCo LegBot] Requesting TROT gait at t=%.3f\n",
                    data->time);
        requestUserParameter(sharedMemory(), "cmpc_gait", 0);
        trotRequested = true;
      }

      if (data->time + 1e-12 >= nextControl) {
        const double speedCmd = speedCommandAtTime(args, data->time);
        fillRobotInputs(model, data, map, sharedMemory(),
                        speedCmd);
        sharedMemory().simulatorIsDone();
        sharedMemory().waitForRobot();
        applyControllerOutput(map, sharedMemory().simToRobot.spiData,
                              spineBoards, data);
        if (args.debug_control && data->time + 1e-12 >= nextDebugPrint) {
          printControlDebug(model, data->time, speedCmd, data, map,
                            sharedMemory().robotToSim.spiCommand, spineBoards);
          if (args.debug_height) {
            printHeightDebug(data->time, speedCmd, data, map);
          }
          nextDebugPrint += 0.5;
        }
        nextControl += args.control_dt;
      }

      mj_step(model, data);
      ++stepCount;
      if (args.viewer && (stepCount % args.render_decimation == 0)) {
        viewer.render(model, data, map);
      }

      if (args.realtime) {
        auto target = wallStart + std::chrono::duration<double>(data->time);
        std::this_thread::sleep_until(target);
      }
    }

    auto wallEnd = std::chrono::steady_clock::now();
    const double wallTime = std::chrono::duration<double>(wallEnd - wallStart).count();
    const double simTime = data->time;
    const double realtimeFactor = wallTime > 0.0 ? simTime / wallTime : 0.0;
    const double finalX = data->qpos[map.base_qpos + 0];
    const double finalZ = data->qpos[map.base_qpos + 2];

    sharedMemory().simToRobot.mode = SimulatorMode::EXIT;
    sharedMemory().simulatorIsDone();
    std::printf("[MuJoCo LegBot] Finished %.3f s. x=%.3f z=%.3f\n", data->time,
                data->qpos[map.base_qpos + 0], data->qpos[map.base_qpos + 2]);
    std::printf(
        "[SUMMARY]\n"
        "sim_time=%.6f\n"
        "wall_time=%.6f\n"
        "realtime_factor=%.6f\n"
        "final_x=%.6f\n"
        "final_z=%.6f\n"
        "viewer_enabled=%d\n"
        "render_decimation=%d\n"
        "debug_flags=control:%d height:%d\n",
        simTime, wallTime, realtimeFactor, finalX, finalZ, args.viewer ? 1 : 0,
        args.render_decimation, args.debug_control ? 1 : 0,
        args.debug_height ? 1 : 0);

    mj_deleteData(data);
    mj_deleteModel(model);
    return 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "[MuJoCo LegBot] ERROR: %s\n", e.what());
    return 1;
  }
}

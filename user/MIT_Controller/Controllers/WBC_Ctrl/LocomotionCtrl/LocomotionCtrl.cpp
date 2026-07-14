#include "LocomotionCtrl.hpp"
#include <cmath>

#ifndef WBC_FORCE_DEBUG
#define WBC_FORCE_DEBUG 0
#endif

#ifndef WBC_CONTACT_DEBUG
#define WBC_CONTACT_DEBUG 0
#endif

namespace {
constexpr double kLegBotWbicFloatingZWeight = 10.0;
constexpr double kLegBotWbicRfZWeight = 0.1;
constexpr int kLegBotGait0TouchdownSlewFrames = 8;
constexpr double kLegBotGait0TouchdownMaxQStep = 0.04;
constexpr double kLegBotGait0TouchdownMaxQdDes = 1.0;

// QPWBC 默认权重（LegBot 会覆盖 z 方向）
constexpr double kDefaultWOri = 300.0;
constexpr double kDefaultWPos = 300.0;
constexpr double kDefaultWFoot = 100.0;
constexpr double kDefaultWFr = 1.0;
constexpr double kDefaultWQddot = 0.001;
constexpr double kMaxFz = 1500.0;
constexpr double kMu = 0.4;
}

template<typename T>
LocomotionCtrl<T>::LocomotionCtrl(FloatingBaseModel<T> model):
  WBC_Ctrl<T>(model)
{
}

template<typename T>
void LocomotionCtrl<T>::_ContactTaskUpdate(void* input, ControlFSMData<T> & data){
  _input_data = static_cast<LocomotionCtrlData<T>* >(input);
  _trunk_mass = data._quadruped->_bodyMass;
  _total_robot_mass = data._quadruped->_totalMass;
  _mpc_mass = data._quadruped->_mpcMass;
  _body_mass = _mpc_mass > T(0) ? _mpc_mass : _trunk_mass;
  _quadruped = data._quadruped;

  _ParameterSetup(data.userParameters);

  _quat_des = ori::rpyToQuat(_input_data->pBody_RPY_des);

  // 构造 double 类型目标（QPWBC 仅 double）
  Vec3<double> p_foot_des_d[4], v_foot_des_d[4], a_foot_des_d[4], Fr_des_d[4];
  for (int leg = 0; leg < 4; ++leg) {
    p_foot_des_d[leg] = _input_data->pFoot_des[leg].template cast<double>();
    v_foot_des_d[leg] = _input_data->vFoot_des[leg].template cast<double>();
    a_foot_des_d[leg] = _input_data->aFoot_des[leg].template cast<double>();
    Fr_des_d[leg] = _input_data->Fr_des[leg].template cast<double>();
  }

  WBCrl::_qp_wbc->setTargets(
      _input_data->pBody_des.template cast<double>(),
      _input_data->vBody_des.template cast<double>(),
      _input_data->aBody_des.template cast<double>(),
      _quat_des.template cast<double>(),
      _input_data->vBody_Ori_des.template cast<double>(),
      p_foot_des_d, v_foot_des_d, a_foot_des_d, Fr_des_d,
      _input_data->contact_state.template cast<double>());

#if WBC_CONTACT_DEBUG
  if (_quadruped && _quadruped->_robotType == RobotType::LEGBOT &&
      WBCrl::_iter % 500 == 0) {
    printf("[WBC_CONTACT] t=%.3f contact_state=[%.2f %.2f %.2f %.2f] "
           "Fr_des_z=[%.3f %.3f %.3f %.3f]\n",
           (double)(WBCrl::_iter * 0.001),
           (double)_input_data->contact_state[0],
           (double)_input_data->contact_state[1],
           (double)_input_data->contact_state[2],
           (double)_input_data->contact_state[3],
           (double)_input_data->Fr_des[0][2],
           (double)_input_data->Fr_des[1][2],
           (double)_input_data->Fr_des[2][2],
           (double)_input_data->Fr_des[3][2]);
  }
#endif
}

template<typename T>
void LocomotionCtrl<T>::_ParameterSetup(const MIT_UserParameters* param){
  // PD 增益（直接从 userParameters 取 Vec3<double>）
  Vec3<double> Kp_ori = param->Kp_ori;
  Vec3<double> Kd_ori = param->Kd_ori;
  Vec3<double> Kp_pos = param->Kp_body;
  Vec3<double> Kd_pos = param->Kd_body;
  Vec3<double> Kp_foot = param->Kp_foot;
  Vec3<double> Kd_foot = param->Kd_foot;

  // 默认权重
  Vec3<double> W_ori = Vec3<double>::Constant(kDefaultWOri);
  Vec3<double> W_pos = Vec3<double>::Constant(kDefaultWPos);
  Vec3<double> W_foot = Vec3<double>::Constant(kDefaultWFoot);
  Vec3<double> W_fr = Vec3<double>::Constant(kDefaultWFr);

  // LegBot 专用调参：z 方向
  if (_quadruped && _quadruped->_robotType == RobotType::LEGBOT) {
    W_pos[2] = kLegBotWbicFloatingZWeight;
    W_fr[2] = kLegBotWbicRfZWeight;
  }

  WBCrl::_qp_wbc->setGains(W_ori, W_pos, W_foot, W_fr, kDefaultWQddot,
                           Kp_ori, Kd_ori, Kp_pos, Kd_pos, Kp_foot, Kd_foot,
                           kMaxFz, kMu);

  // debug 缓存
  _kp_pos_z = T(Kp_pos[2]);
  _kd_pos_z = T(Kd_pos[2]);

  // Leg controller PD 增益（_UpdateLegCMD 使用）
  for (size_t i(0); i < 3; ++i) {
    WBCrl::_Kp_joint[i] = param->Kp_joint[i];
    WBCrl::_Kd_joint[i] = param->Kd_joint[i];
  }
}

template<typename T>
void LocomotionCtrl<T>::_PostComputeCommand(ControlFSMData<T> & data) {
  if(!_input_data) return;

  const int requested_gait = data.userParameters ? data.userParameters->cmpc_gait : -1;
  const bool legbot_gait0 =
      _quadruped && _quadruped->_robotType == RobotType::LEGBOT &&
      (requested_gait == 0 || requested_gait == 10);

  if(!legbot_gait0) {
    _has_prev_contact_state = false;
    for(size_t leg = 0; leg < cheetah::num_leg; ++leg) {
      _touchdown_slew_count[leg] = 0;
      for(size_t jidx = 0; jidx < cheetah::num_leg_joint; ++jidx) {
        const size_t idx = cheetah::num_leg_joint * leg + jidx;
        _last_q_des[leg][jidx] = WBCrl::_des_jpos[idx];
        _last_qd_des[leg][jidx] = WBCrl::_des_jvel[idx];
      }
    }
    return;
  }

  if(!_has_prev_contact_state) {
    _prev_contact_state = _input_data->contact_state;
    _has_prev_contact_state = true;
    for(size_t leg = 0; leg < cheetah::num_leg; ++leg) {
      for(size_t jidx = 0; jidx < cheetah::num_leg_joint; ++jidx) {
        const size_t idx = cheetah::num_leg_joint * leg + jidx;
        _last_q_des[leg][jidx] = WBCrl::_des_jpos[idx];
        _last_qd_des[leg][jidx] = WBCrl::_des_jvel[idx];
      }
    }
    return;
  }

  for(size_t leg = 0; leg < cheetah::num_leg; ++leg) {
    const bool prev_contact = _prev_contact_state[leg] > T(0);
    const bool curr_contact = _input_data->contact_state[leg] > T(0);
    const bool touchdown = !prev_contact && curr_contact;
    if(touchdown) {
      _touchdown_slew_count[leg] = kLegBotGait0TouchdownSlewFrames;
    }

    if(curr_contact && _touchdown_slew_count[leg] > 0) {
      Vec3<T> raw_q_des;
      Vec3<T> raw_qd_des;
      Vec3<T> clamped_q_des;
      Vec3<T> clamped_qd_des;
      Vec3<T> q_step;
      for(size_t jidx = 0; jidx < cheetah::num_leg_joint; ++jidx) {
        const size_t idx = cheetah::num_leg_joint * leg + jidx;
        raw_q_des[jidx] = WBCrl::_des_jpos[idx];
        raw_qd_des[jidx] = WBCrl::_des_jvel[idx];

        T delta = raw_q_des[jidx] - _last_q_des[leg][jidx];
        if(delta > T(kLegBotGait0TouchdownMaxQStep)) {
          delta = T(kLegBotGait0TouchdownMaxQStep);
        } else if(delta < T(-kLegBotGait0TouchdownMaxQStep)) {
          delta = T(-kLegBotGait0TouchdownMaxQStep);
        }
        clamped_q_des[jidx] = _last_q_des[leg][jidx] + delta;
        q_step[jidx] = delta;

        T qd_des = raw_qd_des[jidx];
        if(qd_des > T(kLegBotGait0TouchdownMaxQdDes)) {
          qd_des = T(kLegBotGait0TouchdownMaxQdDes);
        } else if(qd_des < T(-kLegBotGait0TouchdownMaxQdDes)) {
          qd_des = T(-kLegBotGait0TouchdownMaxQdDes);
        }
        clamped_qd_des[jidx] = qd_des;

        WBCrl::_des_jpos[idx] = clamped_q_des[jidx];
        WBCrl::_des_jvel[idx] = clamped_qd_des[jidx];
      }

      if(touchdown) {
        printf("[GAIT0-TOUCHDOWN-WBC-DIAG] leg=%zu prev_contact=%.5f "
               "curr_contact=%.5f slew_frames=%d max_q_step=%.5f "
               "last_qDes=[%.5f %.5f %.5f] raw_qDes=[%.5f %.5f %.5f] "
               "clamped_qDes=[%.5f %.5f %.5f] q_step=[%.5f %.5f %.5f] "
               "raw_qdDes=[%.5f %.5f %.5f] clamped_qdDes=[%.5f %.5f %.5f]\n",
               leg, (double)_prev_contact_state[leg],
               (double)_input_data->contact_state[leg],
               _touchdown_slew_count[leg],
               kLegBotGait0TouchdownMaxQStep,
               (double)_last_q_des[leg][0], (double)_last_q_des[leg][1],
               (double)_last_q_des[leg][2],
               (double)raw_q_des[0], (double)raw_q_des[1],
               (double)raw_q_des[2],
               (double)clamped_q_des[0], (double)clamped_q_des[1],
               (double)clamped_q_des[2],
               (double)q_step[0], (double)q_step[1], (double)q_step[2],
               (double)raw_qd_des[0], (double)raw_qd_des[1],
               (double)raw_qd_des[2],
               (double)clamped_qd_des[0], (double)clamped_qd_des[1],
               (double)clamped_qd_des[2]);
      }
      --_touchdown_slew_count[leg];
    } else if(!curr_contact) {
      _touchdown_slew_count[leg] = 0;
    }
  }

  for(size_t leg = 0; leg < cheetah::num_leg; ++leg) {
    for(size_t jidx = 0; jidx < cheetah::num_leg_joint; ++jidx) {
      const size_t idx = cheetah::num_leg_joint * leg + jidx;
      _last_q_des[leg][jidx] = WBCrl::_des_jpos[idx];
      _last_qd_des[leg][jidx] = WBCrl::_des_jvel[idx];
    }
  }
  _prev_contact_state = _input_data->contact_state;
}

template<typename T>
void LocomotionCtrl<T>::_LCM_PublishData() {
  for(size_t leg(0); leg<4; ++leg){
    _Fr_result[leg] = WBCrl::_qp_wbc->getFrResult(leg).cast<T>();

    if(_input_data->contact_state[leg] > 0.){
      WBCrl::_wbc_data_lcm.contact_est[leg] = 1;
    }else{
      WBCrl::_wbc_data_lcm.contact_est[leg] = 0;
    }
  }

  for(size_t i(0); i<3; ++i){
    WBCrl::_wbc_data_lcm.foot_pos[i] = WBCrl::_model._pGC[linkID::FR][i];
    WBCrl::_wbc_data_lcm.foot_vel[i] = WBCrl::_model._vGC[linkID::FR][i];

    WBCrl::_wbc_data_lcm.foot_pos[i + 3] = WBCrl::_model._pGC[linkID::FL][i];
    WBCrl::_wbc_data_lcm.foot_vel[i + 3] = WBCrl::_model._vGC[linkID::FL][i];

    WBCrl::_wbc_data_lcm.foot_pos[i + 6] = WBCrl::_model._pGC[linkID::HR][i];
    WBCrl::_wbc_data_lcm.foot_vel[i + 6] = WBCrl::_model._vGC[linkID::HR][i];

    WBCrl::_wbc_data_lcm.foot_pos[i + 9] = WBCrl::_model._pGC[linkID::HL][i];
    WBCrl::_wbc_data_lcm.foot_vel[i + 9] = WBCrl::_model._vGC[linkID::HL][i];

    for(size_t leg(0); leg<4; ++leg){
      WBCrl::_wbc_data_lcm.Fr_des[3*leg + i] = _input_data->Fr_des[leg][i];
      WBCrl::_wbc_data_lcm.Fr[3*leg + i] = _Fr_result[leg][i];

      WBCrl::_wbc_data_lcm.foot_pos_cmd[3*leg + i] = _input_data->pFoot_des[leg][i];
      WBCrl::_wbc_data_lcm.foot_vel_cmd[3*leg + i] = _input_data->vFoot_des[leg][i];
      WBCrl::_wbc_data_lcm.foot_acc_cmd[3*leg + i] = _input_data->aFoot_des[leg][i];

      WBCrl::_wbc_data_lcm.jpos_cmd[3*leg + i] = WBCrl::_des_jpos[3*leg + i];
      WBCrl::_wbc_data_lcm.jvel_cmd[3*leg + i] = WBCrl::_des_jvel[3*leg + i];

      WBCrl::_wbc_data_lcm.jpos[3*leg + i] = WBCrl::_state.q[3*leg + i];
      WBCrl::_wbc_data_lcm.jvel[3*leg + i] = WBCrl::_state.qd[3*leg + i];
    }

    WBCrl::_wbc_data_lcm.body_pos_cmd[i] = _input_data->pBody_des[i];
    WBCrl::_wbc_data_lcm.body_vel_cmd[i] = _input_data->vBody_des[i];
    WBCrl::_wbc_data_lcm.body_ori_cmd[i] = _quat_des[i];

    Quat<T> quat = WBCrl::_state.bodyOrientation;
    Mat3<T> Rot = ori::quaternionToRotationMatrix(quat);
    Vec3<T> global_body_vel = Rot.transpose() * WBCrl::_state.bodyVelocity.tail(3);

    WBCrl::_wbc_data_lcm.body_pos[i] = WBCrl::_state.bodyPosition[i];
    WBCrl::_wbc_data_lcm.body_vel[i] = global_body_vel[i];
    WBCrl::_wbc_data_lcm.body_ori[i] = WBCrl::_state.bodyOrientation[i];
    WBCrl::_wbc_data_lcm.body_ang_vel[i] = WBCrl::_state.bodyVelocity[i];
  }

#if WBC_FORCE_DEBUG
  static int forceDebugCounter = 0;
  if ((forceDebugCounter++ % 500) == 0) {
#else
  if (WBCrl::_iter % 500 == 0 && _quadruped &&
      _quadruped->_robotType == RobotType::LEGBOT) {
#endif
    T sumFrDesZ = T(0);
    T sumFrResultZ = T(0);
    const T opCmdZ = T(WBCrl::_qp_wbc->getDesBodyAcc()[2]);
    const T bodyZ = WBCrl::_state.bodyPosition[2];
    const T targetZ = _input_data->pBody_des[2];
    const T posErrZ = targetZ - bodyZ;

    Quat<T> quat = WBCrl::_state.bodyOrientation;
    Mat3<T> rot = ori::quaternionToRotationMatrix(quat);
    SVec<T> currVel = WBCrl::_state.bodyVelocity;
    currVel.tail(3) = rot.transpose() * currVel.tail(3);
    const T velZ = currVel[5];

    const T kpZ = _kp_pos_z;
    const T kdZ = _kd_pos_z;

    T deltaPFootZSum = T(0);
    T qerrSquared = T(0);
    T tauFFSquared = T(0);
    for (size_t leg = 0; leg < cheetah::num_leg; ++leg) {
      Vec3<T> q = WBCrl::_state.q.template segment<3>(3 * leg);
      Vec3<T> qDes = WBCrl::_des_jpos.template segment<3>(3 * leg);
      Vec3<T> pFoot;
      Vec3<T> pFootDes;
      computeLegJacobianAndPosition(*_quadruped, q,
                                    static_cast<Mat3<T>*>(nullptr), &pFoot,
                                    leg);
      computeLegJacobianAndPosition(*_quadruped, qDes,
                                    static_cast<Mat3<T>*>(nullptr),
                                    &pFootDes, leg);
      deltaPFootZSum += pFootDes[2] - pFoot[2];
      for (size_t jidx = 0; jidx < cheetah::num_leg_joint; ++jidx) {
        const size_t idx = cheetah::num_leg_joint * leg + jidx;
        const T qerr = WBCrl::_des_jpos[idx] - WBCrl::_state.q[idx];
        qerrSquared += qerr * qerr;
        tauFFSquared += WBCrl::_tau_ff[idx] * WBCrl::_tau_ff[idx];
      }
    }

    for (size_t leg = 0; leg < 4; ++leg) {
      sumFrDesZ += _input_data->Fr_des[leg][2];
      sumFrResultZ += _Fr_result[leg][2];
    }
#if WBC_FORCE_DEBUG
    printf("[WBC_FORCE_DEBUG] Fr_des_z=[");
    for (size_t leg = 0; leg < 4; ++leg) {
      printf("%s% .3f", leg == 0 ? "" : " ", (double)_input_data->Fr_des[leg][2]);
    }
    printf("] Fr_result_z=[");
    for (size_t leg = 0; leg < 4; ++leg) {
      printf("%s% .3f", leg == 0 ? "" : " ", (double)_Fr_result[leg][2]);
    }
    const T wbcBodyAccZ = T(WBCrl::_qp_wbc->getQddot()[5]);
    printf("] sum_des_z=% .3f sum_result_z=% .3f trunk_mass=%.6f "
           "total_robot_mass=%.6f mpc_mass=%.6f wbc_required_mass=%.6f "
           "mass_g=%.3f op_cmd_z=%.3f wbc_body_acc_z=%.3f "
           "required_Fz=%.3f\n",
           (double)sumFrDesZ, (double)sumFrResultZ, (double)_trunk_mass,
           (double)_total_robot_mass, (double)_mpc_mass, (double)_body_mass,
           (double)(_body_mass * T(9.81)), (double)opCmdZ, (double)wbcBodyAccZ,
           (double)(_body_mass * (T(9.81) + opCmdZ)));
#else
    (void)sumFrDesZ;
#endif
    printf("[VHEIGHT] mode=controller t=%.3f body_z=%.6f target_z=%.6f "
           "pos_err_z=%.6f vel_z=%.6f op_cmd_z=%.6f Kp_z=%.3f Kd_z=%.3f "
           "mpc_mass=%.6f required_Fz=%.3f wbc_sum_Fr_z=%.3f "
           "mujoco_sum_Fn=nan deltaPFoot_z_avg=%.6f qerr_norm=%.6f "
           "tau_ff_norm=%.6f ctrl_norm=nan qvel_z=nan\n",
           (double)(WBCrl::_iter * 0.001), (double)bodyZ, (double)targetZ,
           (double)posErrZ, (double)velZ, (double)opCmdZ, (double)kpZ,
           (double)kdZ, (double)_body_mass,
           (double)(_body_mass * (T(9.81) + opCmdZ)),
           (double)sumFrResultZ, (double)(deltaPFootZSum / T(4)),
           (double)std::sqrt(qerrSquared), (double)std::sqrt(tauFFSquared));
  }
  WBCrl::_wbc_data_lcm.body_ori_cmd[3] = _quat_des[3];
  WBCrl::_wbc_data_lcm.body_ori[3] = WBCrl::_state.bodyOrientation[3];

  WBCrl::_wbcLCM.publish("wbc_lcm_data", &(WBCrl::_wbc_data_lcm) );
}

template class LocomotionCtrl<float>;
template class LocomotionCtrl<double>;

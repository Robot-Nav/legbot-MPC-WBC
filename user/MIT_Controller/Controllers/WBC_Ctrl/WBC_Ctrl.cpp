#include "WBC_Ctrl.hpp"
#include <Utilities/Utilities_print.h>
#include <Utilities/Timer.h>
#include <Configuration.h>

#ifndef HEIGHT_DEBUG
#define HEIGHT_DEBUG 0
#endif

#ifndef LEG_CMD_DEBUG
#define LEG_CMD_DEBUG 0
#endif

template<typename T>
WBC_Ctrl<T>::WBC_Ctrl(FloatingBaseModel<T> model):
  _full_config(cheetah::num_act_joint + 7),
  _tau_ff(cheetah::num_act_joint),
  _des_jpos(cheetah::num_act_joint),
  _des_jvel(cheetah::num_act_joint),
  _wbcLCM(getLcmUrl(255))
{
  _iter = 0;
  _full_config.setZero();

  _model = model;

  // Pinocchio 动力学后端：加载 LegBot URDF
  _qp_wbc = new QPWBC(std::string(THIS_COM) +
                      "models/legbot_description/urdf/legbot_description.urdf");

  _Kp_joint.resize(cheetah::num_leg_joint, 5.);
  _Kd_joint.resize(cheetah::num_leg_joint, 1.5);

  _state.q = DVec<T>::Zero(cheetah::num_act_joint);
  _state.qd = DVec<T>::Zero(cheetah::num_act_joint);
}

template<typename T>
WBC_Ctrl<T>::~WBC_Ctrl(){
  delete _qp_wbc;
}

template <typename T>
void WBC_Ctrl<T>::_ComputeWBC(T dt) {
  _qp_wbc->solve((double)dt);

  // 取回结果（double → T）
  _tau_ff = _qp_wbc->getTauFF().cast<T>();
  _des_jpos = _qp_wbc->getDesJPos().cast<T>();
  _des_jvel = _qp_wbc->getDesJVel().cast<T>();
}

template<typename T>
void WBC_Ctrl<T>::run(void* input, ControlFSMData<T> & data){
  ++_iter;

  const T dt = data.controlParameters->controller_dt;

  // Update Model（FloatingBaseModel + Pinocchio）
  _UpdateModel(data._stateEstimator->getResult(), data._legController->datas);

  // Task & Contact Update（设置 targets/gains）
  _ContactTaskUpdate(input, data);

  // WBC 求解
  _ComputeWBC(dt);
  _PostComputeCommand(data);

  // Update Leg Command
  _UpdateLegCMD(data);

  // LCM publish
  _LCM_PublishData();
}



template<typename T>
void WBC_Ctrl<T>::_UpdateLegCMD(ControlFSMData<T> & data){
  LegControllerCommand<T> * cmd = data._legController->commands;

  for (size_t leg(0); leg < cheetah::num_leg; ++leg) {
    cmd[leg].zero();
    for (size_t jidx(0); jidx < cheetah::num_leg_joint; ++jidx) {
      cmd[leg].tauFeedForward[jidx] = _tau_ff[cheetah::num_leg_joint * leg + jidx];
      cmd[leg].qDes[jidx] = _des_jpos[cheetah::num_leg_joint * leg + jidx];
      cmd[leg].qdDes[jidx] = _des_jvel[cheetah::num_leg_joint * leg + jidx];

        cmd[leg].kpJoint(jidx, jidx) = _Kp_joint[jidx];
        cmd[leg].kdJoint(jidx, jidx) = _Kd_joint[jidx];
    }
  }

  if (data._quadruped->_robotType == RobotType::LEGBOT) {
    const T legbotKp[3] = {T(45), T(55), T(55)};
    const T legbotKd[3] = {T(3.0), T(3.5), T(3.5)};
    for (size_t leg(0); leg < cheetah::num_leg; ++leg) {
      for (size_t jidx(0); jidx < cheetah::num_leg_joint; ++jidx) {
        cmd[leg].kpJoint(jidx, jidx) = legbotKp[jidx];
        cmd[leg].kdJoint(jidx, jidx) = legbotKd[jidx];
      }
    }
  }


  // Knee joint non-flip barrier for the original positive-knee convention.
  // LegBot's MJCF calf joints use a negative working range, so applying this
  // Mini Cheetah barrier would command a positive knee angle and snap the legs
  // straight when WBC enters locomotion.
  if (data._quadruped->_robotType != RobotType::LEGBOT) {
    for(size_t leg(0); leg<4; ++leg){
      if(cmd[leg].qDes[2] < 0.3){
        cmd[leg].qDes[2] = 0.3;
      }
      if(data._legController->datas[leg].q[2] < 0.3){
        T knee_pos = data._legController->datas[leg].q[2];
        cmd[leg].tauFeedForward[2] = 1./(knee_pos * knee_pos + 0.02);
      }
    }
  }

#if LEG_CMD_DEBUG
  if (data._quadruped->_robotType == RobotType::LEGBOT) {
    static int legCmdDebugCounter = 0;
    if ((legCmdDebugCounter++ % 500) == 0) {
      printf("[LEG_CMD_DEBUG]\n");
      for (size_t leg = 0; leg < cheetah::num_leg; ++leg) {
        Vec3<T> q = data._legController->datas[leg].q;
        Vec3<T> qDes = cmd[leg].qDes;
        Vec3<T> pFoot;
        Vec3<T> pFootDes;
        computeLegJacobianAndPosition(*data._quadruped, q,
                                      static_cast<Mat3<T>*>(nullptr), &pFoot,
                                      leg);
        computeLegJacobianAndPosition(*data._quadruped, qDes,
                                      static_cast<Mat3<T>*>(nullptr),
                                      &pFootDes, leg);
        const Vec3<T> deltaPFoot = pFootDes - pFoot;
        printf("  leg %zu q=[% .3f % .3f % .3f] "
               "qDes=[% .3f % .3f % .3f] qd=[% .3f % .3f % .3f] "
               "qdDes=[% .3f % .3f % .3f] tauFF=[% .3f % .3f % .3f] "
               "kp=[% .2f % .2f % .2f] kd=[% .2f % .2f % .2f] "
               "deltaPFoot=[% .4f % .4f % .4f]\n",
               leg,
               (double)data._legController->datas[leg].q[0],
               (double)data._legController->datas[leg].q[1],
               (double)data._legController->datas[leg].q[2],
               (double)cmd[leg].qDes[0], (double)cmd[leg].qDes[1],
               (double)cmd[leg].qDes[2],
               (double)data._legController->datas[leg].qd[0],
               (double)data._legController->datas[leg].qd[1],
               (double)data._legController->datas[leg].qd[2],
               (double)cmd[leg].qdDes[0], (double)cmd[leg].qdDes[1],
               (double)cmd[leg].qdDes[2],
               (double)cmd[leg].tauFeedForward[0],
               (double)cmd[leg].tauFeedForward[1],
               (double)cmd[leg].tauFeedForward[2],
               (double)cmd[leg].kpJoint(0, 0),
               (double)cmd[leg].kpJoint(1, 1),
               (double)cmd[leg].kpJoint(2, 2),
               (double)cmd[leg].kdJoint(0, 0),
               (double)cmd[leg].kdJoint(1, 1),
               (double)cmd[leg].kdJoint(2, 2),
               (double)deltaPFoot[0], (double)deltaPFoot[1],
               (double)deltaPFoot[2]);
      }
    }
  }
#endif
}

template<typename T>
void WBC_Ctrl<T>::_UpdateModel(const StateEstimate<T> & state_est,
    const LegControllerData<T> * leg_data){

  _state.bodyOrientation = state_est.orientation;
  _state.bodyPosition = state_est.position;
  for(size_t i(0); i<3; ++i){
    _state.bodyVelocity[i] = state_est.omegaBody[i];
    _state.bodyVelocity[i+3] = state_est.vBody[i];

    for(size_t leg(0); leg<4; ++leg){
      _state.q[3*leg + i] = leg_data[leg].q[i];
      _state.qd[3*leg + i] = leg_data[leg].qd[i];

      _full_config[3*leg + i + 6] = _state.q[3*leg + i];
    }
  }
  // FloatingBaseModel 更新（保留：_LCM_PublishData 读 _pGC/_vGC）
  _model.setState(_state);
  _model.contactJacobians();
  _model.massMatrix();
  _model.generalizedGravityForce();
  _model.generalizedCoriolisForce();

  _A = _model.getMassMatrix();
  _grav = _model.getGravityForce();
  _coriolis = _model.getCoriolisForce();
  _Ainv = _A.inverse();

  // Pinocchio 动力学更新（QPWBC 内部使用）
  _qp_wbc->updateDynamics(
      _state.bodyOrientation.template cast<double>(),
      _state.bodyPosition.template cast<double>(),
      _state.bodyVelocity.template cast<double>(),
      _state.q.template cast<double>(),
      _state.qd.template cast<double>());

#if HEIGHT_DEBUG
  static int heightDebugCounter = 0;
  if ((heightDebugCounter++ % 500) == 0) {
    printf("[HEIGHT_DEBUG WBC] current_z=%.6f model_state_z=%.6f "
           "estimator_z=%.6f q0=[%.3f %.3f %.3f]\n",
           (double)_state.bodyPosition[2],
           (double)_model._state.bodyPosition[2],
           (double)state_est.position[2], (double)_state.q[0],
           (double)_state.q[1], (double)_state.q[2]);
  }
#endif
}


template class WBC_Ctrl<float>;
template class WBC_Ctrl<double>;

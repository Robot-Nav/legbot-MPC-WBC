#include "PinocchioDynamics.hpp"

#include <iostream>

PinocchioDynamics::PinocchioDynamics(const std::string& urdf_path) {
  pinocchio::urdf::buildModel(urdf_path, pinocchio::JointModelFreeFlyer(),
                              _model);

  _q = Eigen::VectorXd::Zero(_model.nq);
  _v = Eigen::VectorXd::Zero(_model.nv);

  // URDF 用 RR/RL，linkID 用 HR/HL：此处做映射
  //   leg 0 = FR, 1 = FL, 2 = HR(URDF:RR), 3 = HL(URDF:RL)
  static const char* kFootFrameNames[kNumLeg] = {"FR_foot", "FL_foot",
                                                 "RR_foot", "RL_foot"};
  for (int leg = 0; leg < kNumLeg; ++leg) {
    _foot_ids[leg] = _model.getFrameId(kFootFrameNames[leg]);
    if (_foot_ids[leg] >= _model.frames.size()) {
      std::cerr << "[PinocchioDynamics] 足端帧 " << kFootFrameNames[leg]
                << " 未找到，URDF: " << urdf_path << std::endl;
    }
  }
  _base_id = _model.getFrameId("base");

  _M.setZero();
  _cori.setZero();
  _grav.setZero();
  for (int leg = 0; leg < kNumLeg; ++leg) {
    _Jc_foot[leg].setZero();
    _Jcdqd_foot[leg].setZero();
    _p_foot[leg].setZero();
    _v_foot[leg].setZero();
  }
  _Jc_body.setZero();
  _Jcdqd_body.setZero();
  _R_body.setIdentity();
  _p_body.setZero();
  _J6_tmp.setZero();
}

void PinocchioDynamics::updateState(const Quat<double>& ori_wxyz,
                                    const Vec3<double>& pos,
                                    const SVec<double>& body_vel,
                                    const Vec12<double>& q_joint,
                                    const Vec12<double>& qd_joint) {
  // 约定转换：四元数 [w,x,y,z] → Pinocchio [x,y,z,w]
  _q(0) = pos(0);
  _q(1) = pos(1);
  _q(2) = pos(2);
  _q(3) = ori_wxyz(1);
  _q(4) = ori_wxyz(2);
  _q(5) = ori_wxyz(3);
  _q(6) = ori_wxyz(0);
  _q.tail<12>() = q_joint;

  // 浮基速度 [ω;v] → Pinocchio [v;ω]（body 系）
  _v(0) = body_vel(3);
  _v(1) = body_vel(4);
  _v(2) = body_vel(5);
  _v(3) = body_vel(0);
  _v(4) = body_vel(1);
  _v(5) = body_vel(2);
  _v.tail<12>() = qd_joint;

  // 动力学量
  pinocchio::crba(_model, _data, _q);
  _M = _data.M.selfadjointView<Eigen::Upper>();

  pinocchio::computeGeneralizedGravity(_model, _data, _q);
  _grav = _data.g;

  pinocchio::computeCoriolisMatrix(_model, _data, _q, _v);
  _cori = _data.C * _v;

  // 运动学 + Jacobian：
  //   forwardKinematics(q,v) 填 oMi/v
  //   computeJointJacobians(q) 填 J
  //   computeJointJacobiansTimeVariation(q,v) 填 dJ（getFrameJacobianTimeVariation 依赖）
  pinocchio::forwardKinematics(_model, _data, _q, _v);
  pinocchio::computeJointJacobians(_model, _data, _q);
  pinocchio::computeJointJacobiansTimeVariation(_model, _data, _q, _v);
  pinocchio::updateFramePlacements(_model, _data);

  // 基座位姿
  _R_body = _data.oMf[_base_id].rotation();
  _p_body = _data.oMf[_base_id].translation();

  // 身体 Jacobian（LOCAL_WORLD_ALIGNED，行序 [ω;v]）
  _J6_tmp.setZero();
  pinocchio::getFrameJacobian(_model, _data, _base_id,
                              pinocchio::LOCAL_WORLD_ALIGNED, _J6_tmp);
  _Jc_body = _J6_tmp;

  _J6_tmp.setZero();
  pinocchio::getFrameJacobianTimeVariation(_model, _data, _base_id,
                                           pinocchio::LOCAL_WORLD_ALIGNED,
                                           _J6_tmp);
  _Jcdqd_body = _J6_tmp * _v;

  // 足端：取线性部分（WORLD）
  for (int leg = 0; leg < kNumLeg; ++leg) {
    const auto& Mf = _data.oMf[_foot_ids[leg]];
    _p_foot[leg] = Mf.translation();

    _J6_tmp.setZero();
    pinocchio::getFrameJacobian(_model, _data, _foot_ids[leg],
                                pinocchio::WORLD, _J6_tmp);
    _Jc_foot[leg] = _J6_tmp.bottomRows<3>();
    _v_foot[leg] = _Jc_foot[leg] * _v;

    _J6_tmp.setZero();
    pinocchio::getFrameJacobianTimeVariation(_model, _data, _foot_ids[leg],
                                             pinocchio::WORLD, _J6_tmp);
    _Jcdqd_foot[leg] = (_J6_tmp * _v).tail<3>();
  }
}

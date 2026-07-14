#include "QPWBC.hpp"

#include <iostream>

QPWBC::QPWBC(const std::string& urdf_path) : _pin(urdf_path) {
  // 默认权重（计划 §7 起步值，LocomotionCtrl 会按 LegBot 调参覆盖）
  _W_ori = Vec3<double>(300.0, 300.0, 300.0);
  _W_pos = Vec3<double>(300.0, 300.0, 300.0);
  _W_foot = Vec3<double>(100.0, 100.0, 100.0);
  _W_fr = Vec3<double>(1.0, 1.0, 1.0);
  _W_qddot = 0.001;

  _tau_ff.setZero();
  _des_jpos.setZero();
  _des_jvel.setZero();
  _qddot.setZero();
  _des_body_acc.setZero();
  _q_joint_curr.setZero();
  _qd_joint_curr.setZero();
  _omega_body.setZero();
  _v_body.setZero();
  for (int i = 0; i < 4; ++i) _Fr_result[i].setZero();

  _P_dense.setZero();
  _A_dense.setZero();
  _q_vec.setZero();
  _l_vec.setZero();
  _u_vec.setZero();
  _x_warm.setZero();

  buildFrictionCone();
}

void QPWBC::updateDynamics(const Quat<double>& ori_wxyz, const Vec3<double>& pos,
                           const SVec<double>& body_vel, const Vec12<double>& q_joint,
                           const Vec12<double>& qd_joint) {
  _pin.updateState(ori_wxyz, pos, body_vel, q_joint, qd_joint);
  _q_joint_curr = q_joint;
  _qd_joint_curr = qd_joint;
  // MIT 约定 body_vel = [ω(3); v(3)] body 系
  _omega_body = body_vel.head<3>();
  _v_body = body_vel.tail<3>();
}

void QPWBC::setTargets(const Vec3<double>& p_body_des,
                       const Vec3<double>& v_body_des,
                       const Vec3<double>& a_body_des,
                       const Quat<double>& ori_des,
                       const Vec3<double>& vel_ori_des,
                       const Vec3<double> p_foot_des[4],
                       const Vec3<double> v_foot_des[4],
                       const Vec3<double> a_foot_des[4],
                       const Vec3<double> Fr_des[4],
                       const Vec4<double>& contact_state) {
  _p_body_des = p_body_des;
  _v_body_des = v_body_des;
  _a_body_des = a_body_des;
  _ori_des = ori_des;
  _vel_ori_des = vel_ori_des;
  for (int i = 0; i < 4; ++i) {
    _p_foot_des[i] = p_foot_des[i];
    _v_foot_des[i] = v_foot_des[i];
    _a_foot_des[i] = a_foot_des[i];
    _Fr_des[i] = Fr_des[i];
  }
  _contact_state = contact_state;
}

void QPWBC::setGains(const Vec3<double>& W_ori, const Vec3<double>& W_pos,
                     const Vec3<double>& W_foot, const Vec3<double>& W_fr,
                     double W_qddot,
                     const Vec3<double>& Kp_ori, const Vec3<double>& Kd_ori,
                     const Vec3<double>& Kp_pos, const Vec3<double>& Kd_pos,
                     const Vec3<double>& Kp_foot, const Vec3<double>& Kd_foot,
                     double max_Fz, double mu) {
  _W_ori = W_ori;
  _W_pos = W_pos;
  _W_foot = W_foot;
  _W_fr = W_fr;
  _W_qddot = W_qddot;
  _Kp_ori = Kp_ori;
  _Kd_ori = Kd_ori;
  _Kp_pos = Kp_pos;
  _Kd_pos = Kd_pos;
  _Kp_foot = Kp_foot;
  _Kd_foot = Kd_foot;
  _max_Fz = max_Fz;
  _mu = mu;
  buildFrictionCone();
}

void QPWBC::buildFrictionCone() {
  // 与 SingleContact 一致：Fz>=0; Fx±μFz>=0; Fy±μFz>=0; Fz<=max_Fz
  _Uf.setZero();
  _Uf(0, 2) = 1.0;
  _Uf(1, 0) = 1.0;  _Uf(1, 2) = _mu;
  _Uf(2, 0) = -1.0; _Uf(2, 2) = _mu;
  _Uf(3, 1) = 1.0;  _Uf(3, 2) = _mu;
  _Uf(4, 1) = -1.0; _Uf(4, 2) = _mu;
  _Uf(5, 2) = -1.0;
}

void QPWBC::buildQP(double dt) {
  (void)dt;

  // 1. 接触腿列表
  _nc = 0;
  for (int leg = 0; leg < 4; ++leg) {
    if (_contact_state[leg] > 0.0) {
      _contact_legs[_nc++] = leg;
    }
  }

  const int n = 18 + 3 * _nc;
  const int m_eq = 6 + 3 * _nc;
  const int m = m_eq + 6 * _nc;

  // 2. 清零 dense 缓冲（按最大维度清，OSQP 只读前 n 列/m 行）
  _P_dense.setZero();
  _A_dense.setZero();
  _q_vec.setZero();
  _l_vec.setZero();
  _u_vec.setZero();

  // 3. 任务 Jacobian 和期望加速度
  const auto& Jc_body = _pin.getBodyJacobian();         // 6×18, LOCAL_WORLD_ALIGNED
  const auto& Jcdqd_body = _pin.getBodyJcDotQdot();     // 6
  const Mat3<double>& R_body = _pin.getBodyRotation();  // R_wb（world←body）
  const Vec3<double>& p_body = _pin.getBodyPosition();

  // body_ori：Jt = body Jacobian 角速度行（WORLD 系），等价 MIT [R_wb, 0]
  Eigen::Matrix<double, 3, 18> Jt_ori = Jc_body.topRows<3>();
  Vec3<double> Jcdqd_ori = Jcdqd_body.head<3>();
  // ori_err = log3(R_wb_des · R_bw_body)（与 MIT quaternionToso3(q_des⊗q_body⁻¹) 一致）
  Mat3<double> R_des = ori::quaternionToRotationMatrix(_ori_des).transpose();  // R_wb_des
  Vec3<double> ori_err = pinocchio::log3(R_des * R_body.transpose());
  // vel_ori_des 是 body 系，转世界系
  Vec3<double> omega_world = R_body * _omega_body;
  Vec3<double> omega_des_world = R_body * _vel_ori_des;
  Vec3<double> xddot_des_ori =
      _Kp_ori.cwiseProduct(ori_err) + _Kd_ori.cwiseProduct(omega_des_world - omega_world);

  // body_pos：Jt = body Jacobian 线速度行（WORLD 系），等价 MIT [0, R_wb, 0]
  Eigen::Matrix<double, 3, 18> Jt_pos = Jc_body.bottomRows<3>();
  Vec3<double> Jcdqd_pos = Jcdqd_body.tail<3>();
  Vec3<double> v_world = R_body * _v_body;
  Vec3<double> xddot_des_pos =
      _Kp_pos.cwiseProduct(_p_body_des - p_body) +
      _Kd_pos.cwiseProduct(_v_body_des - v_world) + _a_body_des;
  _des_body_acc = xddot_des_pos;

  // 4. P[0:18, 0:18] = Σ Wi·Jiᵀ·Ji + W_qddot·I
  Mat18<double> P_top = Mat18<double>::Zero();
  P_top += Jt_ori.transpose() * _W_ori.asDiagonal() * Jt_ori;
  P_top += Jt_pos.transpose() * _W_pos.asDiagonal() * Jt_pos;
  for (int leg = 0; leg < 4; ++leg) {
    if (_contact_state[leg] <= 0.0) {  // 摆动腿
      const auto& Jf = _pin.getFootJacobian(leg);
      P_top += Jf.transpose() * _W_foot.asDiagonal() * Jf;
    }
  }
  P_top += _W_qddot * Mat18<double>::Identity();
  _P_dense.topLeftCorner<18, 18>() = P_top;

  // 5. P[18:, 18:] = block_diag(W_fr)
  for (int k = 0; k < _nc; ++k) {
    _P_dense.block<3, 3>(18 + 3 * k, 18 + 3 * k) = _W_fr.asDiagonal();
  }

  // 6. q[0:18] = Σ Wi·Jiᵀ·(Jdqd_i − xddot_des_i)
  Vec18<double> q_top = Vec18<double>::Zero();
  q_top += Jt_ori.transpose() * _W_ori.asDiagonal() * (Jcdqd_ori - xddot_des_ori);
  q_top += Jt_pos.transpose() * _W_pos.asDiagonal() * (Jcdqd_pos - xddot_des_pos);
  for (int leg = 0; leg < 4; ++leg) {
    if (_contact_state[leg] <= 0.0) {
      const auto& Jf = _pin.getFootJacobian(leg);
      const auto& Jcdqd_f = _pin.getFootJcDotQdot(leg);
      const Vec3<double>& pf = _pin.getFootPos(leg);
      const Vec3<double>& vf = _pin.getFootVel(leg);
      Vec3<double> xddot_des_foot =
          _Kp_foot.cwiseProduct(_p_foot_des[leg] - pf) +
          _Kd_foot.cwiseProduct(_v_foot_des[leg] - vf) + _a_foot_des[leg];
      q_top += Jf.transpose() * _W_foot.asDiagonal() * (Jcdqd_f - xddot_des_foot);
    }
  }
  _q_vec.head<18>() = q_top;

  // 7. q[18:] = −W_fr · Fr_des
  for (int k = 0; k < _nc; ++k) {
    int leg = _contact_legs[k];
    _q_vec.segment<3>(18 + 3 * k) = -_W_fr.cwiseProduct(_Fr_des[leg]);
  }

  // 8. 堆叠接触足 Jacobian（3nc×18）和 Jcdqd
  Eigen::Matrix<double, 12, 18> Jc_contact = Eigen::Matrix<double, 12, 18>::Zero();
  Vec3<double> Jcdqd_contact[4];
  for (int k = 0; k < _nc; ++k) {
    int leg = _contact_legs[k];
    Jc_contact.block<3, 18>(3 * k, 0) = _pin.getFootJacobian(leg);
    Jcdqd_contact[k] = _pin.getFootJcDotQdot(leg);
  }

  // 9. 等式约束
  const Mat18<double>& M = _pin.getMassMatrix();
  const Vec18<double>& cori = _pin.getCoriolis();
  const Vec18<double>& grav = _pin.getGravity();

  // (a) 浮基动力学(6): [M[0:6,:], −Jc_contactᵀ[0:6,:]]·x = −(cori+grav)[0:6]
  _A_dense.block<6, 18>(0, 0) = M.topRows<6>();
  for (int k = 0; k < _nc; ++k) {
    // Jc_contact[3k:3k+3, 0:6]ᵀ → A[0:6, 18+3k:18+3k+3]
    _A_dense.block<6, 3>(0, 18 + 3 * k) = -Jc_contact.block<3, 6>(3 * k, 0).transpose();
  }
  _l_vec.head<6>() = -(cori + grav).head<6>();
  _u_vec.head<6>() = -(cori + grav).head<6>();

  // (b) 接触一致性(3nc): [Jc_contact, 0]·x = −Jcdqd
  for (int k = 0; k < _nc; ++k) {
    _A_dense.block<3, 18>(6 + 3 * k, 0) = Jc_contact.block<3, 18>(3 * k, 0);
    _l_vec.segment<3>(6 + 3 * k) = -Jcdqd_contact[k];
    _u_vec.segment<3>(6 + 3 * k) = -Jcdqd_contact[k];
  }

  // 10. 不等式约束(6nc): [0, Uf_block]·x >= ieq_vec
  for (int k = 0; k < _nc; ++k) {
    int row = m_eq + 6 * k;
    _A_dense.block<6, 3>(row, 18 + 3 * k) = _Uf;
    _l_vec.segment<6>(row) << 0.0, 0.0, 0.0, 0.0, 0.0, -_max_Fz;
    _u_vec.segment<6>(row).setConstant(OSQP_INFTY);
  }
}

bool QPWBC::solve(double dt) {
  buildQP(dt);

  const int n = 18 + 3 * _nc;
  const int m = 6 + 3 * _nc + 6 * _nc;

  bool ok;
  if (_last_nc != _nc) {
    // nc 变化，重新 setup
    ok = _osqp.setup(n, m, _P_dense.data(), _A_dense.data(),
                     _q_vec.data(), _l_vec.data(), _u_vec.data());
    _x_warm.setZero();
    _last_nc = _nc;
  } else {
    ok = _osqp.update(_P_dense.data(), _A_dense.data(), _q_vec.data(),
                      _l_vec.data(), _u_vec.data(), _x_warm.data());
  }

  if (!ok) {
    std::cerr << "[QPWBC] OSQP setup/update failed (nc=" << _nc << ")" << std::endl;
    return false;
  }

  if (!_osqp.solve()) {
    std::cerr << "[QPWBC] OSQP solve failed, status=" << _osqp.status()
              << " (nc=" << _nc << ")" << std::endl;
    return false;
  }

  // 缓存解作为下次 warm start
  const c_float* sol = _osqp.solutionX();
  if (sol) {
    for (int i = 0; i < n; ++i) _x_warm[i] = sol[i];
  }

  extractResults(dt);
  return true;
}

void QPWBC::extractResults(double dt) {
  const c_float* sol = _osqp.solutionX();
  if (!sol) {
    _tau_ff.setZero();
    _des_jpos = _q_joint_curr;
    _des_jvel = _qd_joint_curr;
    _qddot.setZero();
    for (int i = 0; i < 4; ++i) _Fr_result[i].setZero();
    return;
  }

  // qddot(18)
  for (int i = 0; i < 18; ++i) _qddot[i] = sol[i];

  // Fr 按腿回填（接触腿从 QP 解，摆动腿填 0）
  for (int k = 0; k < _nc; ++k) {
    int leg = _contact_legs[k];
    for (int i = 0; i < 3; ++i) {
      _Fr_result[leg][i] = sol[18 + 3 * k + i];
    }
  }
  for (int leg = 0; leg < 4; ++leg) {
    if (_contact_state[leg] <= 0.0) _Fr_result[leg].setZero();
  }

  // tau = (M·qddot + cori + grav − Jcᵀ·Fr)[6:18]
  const Mat18<double>& M = _pin.getMassMatrix();
  const Vec18<double>& cori = _pin.getCoriolis();
  const Vec18<double>& grav = _pin.getGravity();

  Vec18<double> JcTFr = Vec18<double>::Zero();
  for (int k = 0; k < _nc; ++k) {
    int leg = _contact_legs[k];
    const auto& Jf = _pin.getFootJacobian(leg);
    Vec3<double> Fr_leg;
    for (int i = 0; i < 3; ++i) Fr_leg[i] = sol[18 + 3 * k + i];
    JcTFr += Jf.transpose() * Fr_leg;
  }

  Vec18<double> tau_full = M * _qddot + cori + grav - JcTFr;
  _tau_ff = tau_full.tail<12>();

  // des_jvel = qd_joint + qddot[6:18]·dt
  _des_jvel = _qd_joint_curr + _qddot.tail<12>() * dt;
  // des_jpos = q_joint + des_jvel·dt
  _des_jpos = _q_joint_curr + _des_jvel * dt;
}

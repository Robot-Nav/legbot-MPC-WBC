#ifndef WBC_QP_WBC_HPP
#define WBC_QP_WBC_HPP

// 统一 QP-WBC：基于 Pinocchio 动力学 + OSQP 单一 QP
// 合并 KinWBC（运动学零空间投影）+ WBIC（浮基冲量控制）为单一 QP
//
// 决策变量 x = [qddot(18); Fr(3·nc)]，nc=支撑腿数
// 代价：软优先级 Σ Wi·||Ji·qddot + Jdqd_i − xddot_des_i||²
//                + W_fr·||Fr−Fr_des||² + W_qddot·||qddot||²
// 等式：浮基动力学(6) + 接触一致性(3·nc)
// 不等式：摩擦锥(6·nc, μ=0.4)
//
// 仅 double；WBC_Ctrl<T> 在边界 cast<double>()/cast<T>()

#include <string>
#include "cppTypes.h"
#include "Math/orientation_tools.h"
#include "PinocchioDynamics.hpp"
#include "OSQPWrapper.hpp"

class QPWBC {
 public:
  explicit QPWBC(const std::string& urdf_path);
  ~QPWBC() = default;

  // 1. 更新动力学状态（MIT 约定：ori=[w,x,y,z], body_vel=[ω;v] body 系）
  void updateDynamics(const Quat<double>& ori_wxyz, const Vec3<double>& pos,
                      const SVec<double>& body_vel, const Vec12<double>& q_joint,
                      const Vec12<double>& qd_joint);

  // 2. 设置目标
  //    ori_des: [w,x,y,z]
  //    vel_ori_des: body 系角速度（与 BodyOriTask 一致）
  //    v_body_des / v_foot_des: 世界系（与 BodyPosTask/LinkPosTask 一致）
  void setTargets(const Vec3<double>& p_body_des,
                  const Vec3<double>& v_body_des,
                  const Vec3<double>& a_body_des,
                  const Quat<double>& ori_des,
                  const Vec3<double>& vel_ori_des,
                  const Vec3<double> p_foot_des[4],
                  const Vec3<double> v_foot_des[4],
                  const Vec3<double> a_foot_des[4],
                  const Vec3<double> Fr_des[4],
                  const Vec4<double>& contact_state);

  // 3. 设置增益（Vec3 权重/PD 支持 per-axis 调参，如 LegBot z 方向）
  void setGains(const Vec3<double>& W_ori, const Vec3<double>& W_pos,
                const Vec3<double>& W_foot, const Vec3<double>& W_fr,
                double W_qddot,
                const Vec3<double>& Kp_ori, const Vec3<double>& Kd_ori,
                const Vec3<double>& Kp_pos, const Vec3<double>& Kd_pos,
                const Vec3<double>& Kp_foot, const Vec3<double>& Kd_foot,
                double max_Fz, double mu = 0.4);

  // 4. 求解（nc 不变→update+warmstart；变→setup）
  bool solve(double dt);

  // 5. 结果
  const Vec12<double>& getTauFF() const { return _tau_ff; }
  const Vec12<double>& getDesJPos() const { return _des_jpos; }
  const Vec12<double>& getDesJVel() const { return _des_jvel; }
  const Vec3<double>& getFrResult(int leg) const { return _Fr_result[leg]; }
  const Vec18<double>& getQddot() const { return _qddot; }
  const Vec3<double>& getDesBodyAcc() const { return _des_body_acc; }

 private:
  PinocchioDynamics _pin;
  OSQPWrapper _osqp;
  int _last_nc = -1;

  // 状态缓存（updateDynamics 时存）
  Vec12<double> _q_joint_curr, _qd_joint_curr;
  Vec3<double> _omega_body, _v_body;  // body 系

  // 目标缓存
  Vec3<double> _p_body_des, _v_body_des, _a_body_des;
  Quat<double> _ori_des;
  Vec3<double> _vel_ori_des;
  Vec3<double> _p_foot_des[4], _v_foot_des[4], _a_foot_des[4], _Fr_des[4];
  Vec4<double> _contact_state;

  // 增益
  Vec3<double> _W_ori, _W_pos, _W_foot, _W_fr;
  double _W_qddot = 0.001;
  Vec3<double> _Kp_ori, _Kd_ori, _Kp_pos, _Kd_pos, _Kp_foot, _Kd_foot;
  double _max_Fz = 1500.0, _mu = 0.4;

  // 结果
  Vec12<double> _tau_ff, _des_jpos, _des_jvel;
  Vec3<double> _Fr_result[4];
  Vec18<double> _qddot;
  Vec3<double> _des_body_acc;

  // 接触腿索引（每拍重算）
  int _contact_legs[4];
  int _nc = 0;

  // QP dense 缓冲（最大 nc=4: n=30, m=42）
  static constexpr int kNmax = 30;  // 18 + 3*4
  static constexpr int kMmax = 42;  // 6 + 3*4 + 6*4
  Eigen::Matrix<double, kNmax, kNmax> _P_dense;
  Eigen::Matrix<double, kMmax, kNmax> _A_dense;
  Eigen::Matrix<double, kNmax, 1> _q_vec, _x_warm;
  Eigen::Matrix<double, kMmax, 1> _l_vec, _u_vec;

  // 摩擦锥 Uf(6×3)，与 SingleContact 一致
  Eigen::Matrix<double, 6, 3> _Uf;

  void buildFrictionCone();
  void buildQP(double dt);
  void extractResults(double dt);
};

#endif  // WBC_QP_WBC_HPP

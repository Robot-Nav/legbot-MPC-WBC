#ifndef WBC_PINOCCHIO_DYNAMICS_HPP
#define WBC_PINOCCHIO_DYNAMICS_HPP

// 基于 Pinocchio 的浮基动力学后端
// 仅 double，供 QPWBC 内部使用；WBC_Ctrl<T> 在边界 cast<double>()/cast<T>()
//
// 约定转换（MIT Cheetah → Pinocchio）：
//   四元数 [w,x,y,z] → [x,y,z,w]
//   浮基速度 [ω(3);v(3)] → [v(3);ω(3)]（body coords）
//   足端帧名映射 linkID: FR→FR_foot, FL→FL_foot, HR→RR_foot, HL→RL_foot

#include <string>
#include <Eigen/Dense>
#include "cppTypes.h"

#include "pinocchio/multibody.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/spatial.hpp"

// leg 索引：0=FR, 1=FL, 2=HR(RR), 3=HL(RL)，与 linkID 一致
constexpr int kNumLeg = 4;

class PinocchioDynamics {
 public:
  // urdf_path: legbot_description.urdf 绝对路径
  explicit PinocchioDynamics(const std::string& urdf_path);
  ~PinocchioDynamics() = default;

  // 用 MIT 约定状态更新模型，缓存所有动力学量
  //   ori_wxyz: [w,x,y,z]
  //   pos:      世界系基座位置
  //   body_vel: [ω(3); v(3)]，body 系
  //   q_joint:  12 关节角
  //   qd_joint: 12 关节角速度
  void updateState(const Quat<double>& ori_wxyz, const Vec3<double>& pos,
                   const SVec<double>& body_vel, const Vec12<double>& q_joint,
                   const Vec12<double>& qd_joint);

  // 动力学量（nq=19, nv=18）
  const Mat18<double>& getMassMatrix() const { return _M; }
  const Vec18<double>& getCoriolis() const { return _cori; }
  const Vec18<double>& getGravity() const { return _grav; }

  // 足端：线性 Jacobian(3×18, WORLD)、JcDotQdot(3)、位置(3, WORLD)、速度(3, WORLD)
  const Eigen::Matrix<double, 3, 18>& getFootJacobian(int leg) const {
    return _Jc_foot[leg];
  }
  const Vec3<double>& getFootJcDotQdot(int leg) const {
    return _Jcdqd_foot[leg];
  }
  const Vec3<double>& getFootPos(int leg) const { return _p_foot[leg]; }
  const Vec3<double>& getFootVel(int leg) const { return _v_foot[leg]; }

  // 身体帧：6×18 Jacobian(LOCAL_WORLD_ALIGNED, 行序[ω;v])、JcDotQdot(6)
  const Eigen::Matrix<double, 6, 18>& getBodyJacobian() const {
    return _Jc_body;
  }
  const Vec6<double>& getBodyJcDotQdot() const { return _Jcdqd_body; }

  // 基座旋转矩阵（世界系←body 系坐标变换）
  const Mat3<double>& getBodyRotation() const { return _R_body; }
  const Vec3<double>& getBodyPosition() const { return _p_body; }

 private:
  pinocchio::Model _model;
  pinocchio::Data _data;

  pinocchio::FrameIndex _foot_ids[kNumLeg];
  pinocchio::FrameIndex _base_id;

  // Pinocchio 配置向量 q(19)/速度 v(18)
  Eigen::VectorXd _q, _v;

  // 缓存结果
  Mat18<double> _M;
  Vec18<double> _cori, _grav;
  Eigen::Matrix<double, 3, 18> _Jc_foot[kNumLeg];
  Vec3<double> _Jcdqd_foot[kNumLeg];
  Vec3<double> _p_foot[kNumLeg], _v_foot[kNumLeg];
  Eigen::Matrix<double, 6, 18> _Jc_body;
  Vec6<double> _Jcdqd_body;
  Mat3<double> _R_body;
  Vec3<double> _p_body;

  // 6×18 临时缓冲（getFrameJacobian 需 6 行）
  Eigen::Matrix<double, 6, 18> _J6_tmp;
};

#endif  // WBC_PINOCCHIO_DYNAMICS_HPP

#ifndef CHEETAH_SOFTWARE_CONVEXMPCLOCOMOTION_H
#define CHEETAH_SOFTWARE_CONVEXMPCLOCOMOTION_H

#include <Controllers/FootSwingTrajectory.h>
#include <FSM_States/ControlFSMData.h>
#include <SparseCMPC/SparseCMPC.h>
#include "cppTypes.h"
#include "Gait.h"

#include <cstdio>

using Eigen::Array4f;
using Eigen::Array4i;


template<typename T>
struct CMPC_Result {
  LegControllerCommand<T> commands[4];
  Vec4<T> contactPhase;
};

struct CMPC_Jump {
  static constexpr int START_SEG = 6;
  static constexpr int END_SEG = 0;
  static constexpr int END_COUNT = 2;
  bool jump_pending = false;
  bool jump_in_progress = false;
  bool pressed = false;
  int seen_end_count = 0;
  int last_seg_seen = 0;
  int jump_wait_counter = 0;

  void debug(int seg) {
    (void)seg;
    //printf("[%d] pending %d running %d\n", seg, jump_pending, jump_in_progress);
  }

  void trigger_pressed(int seg, bool trigger) {
    (void)seg;
    if(!pressed && trigger) {
      if(!jump_pending && !jump_in_progress) {
        jump_pending = true;
        //printf("jump pending @ %d\n", seg);
      }
    }
    pressed = trigger;
  }

  bool should_jump(int seg) {
    debug(seg);

    if(jump_pending && seg == START_SEG) {
      jump_pending = false;
      jump_in_progress = true;
      //printf("jump begin @ %d\n", seg);
      seen_end_count = 0;
      last_seg_seen = seg;
      return true;
    }

    if(jump_in_progress) {
      if(seg == END_SEG && seg != last_seg_seen) {
        seen_end_count++;
        if(seen_end_count == END_COUNT) {
          seen_end_count = 0;
          jump_in_progress = false;
          //printf("jump end @ %d\n", seg);
          last_seg_seen = seg;
          return false;
        }
      }
      last_seg_seen = seg;
      return true;
    }

    last_seg_seen = seg;
    return false;
  }
};

struct ConvexMPCDebugSnapshot {
  bool valid = false;
  int gaitNumber = -1;
  int currentGait = -1;
  int gaitPhase = -1;
  int iterationCounter = 0;
  Vec4<float> contactStates = Vec4<float>::Zero();
  Vec4<float> swingStates = Vec4<float>::Zero();
  Vec4<float> seContactState = Vec4<float>::Zero();
  Vec4<float> swingTimes = Vec4<float>::Zero();
  Vec4<float> swingTimeRemaining = Vec4<float>::Zero();
  Vec3<float> pFoot[4] = {Vec3<float>::Zero(), Vec3<float>::Zero(),
                          Vec3<float>::Zero(), Vec3<float>::Zero()};
  Vec3<float> pFootDes[4] = {Vec3<float>::Zero(), Vec3<float>::Zero(),
                             Vec3<float>::Zero(), Vec3<float>::Zero()};
  Vec3<float> vFootDes[4] = {Vec3<float>::Zero(), Vec3<float>::Zero(),
                             Vec3<float>::Zero(), Vec3<float>::Zero()};
  Vec3<float> swingP0[4] = {Vec3<float>::Zero(), Vec3<float>::Zero(),
                            Vec3<float>::Zero(), Vec3<float>::Zero()};
  Vec3<float> swingPf[4] = {Vec3<float>::Zero(), Vec3<float>::Zero(),
                            Vec3<float>::Zero(), Vec3<float>::Zero()};
  Vec3<float> FrDes[4] = {Vec3<float>::Zero(), Vec3<float>::Zero(),
                          Vec3<float>::Zero(), Vec3<float>::Zero()};
  Eigen::Vector4i mpcTableNow = Eigen::Vector4i::Zero();
  Eigen::Vector4i firstSwing = Eigen::Vector4i::Zero();
};


class ConvexMPCLocomotion {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ConvexMPCLocomotion(float _dt, int _iterations_between_mpc, MIT_UserParameters* parameters);
  void initialize();
  void initializeStandingTargetForLegBot(ControlFSMData<float>& data);

  template<typename T>
  void run(ControlFSMData<T>& data);
  bool currently_jumping = false;

  Vec3<float> pBody_des;
  Vec3<float> vBody_des;
  Vec3<float> aBody_des;

  Vec3<float> pBody_RPY_des;
  Vec3<float> vBody_Ori_des;

  Vec3<float> pFoot_des[4];
  Vec3<float> vFoot_des[4];
  Vec3<float> aFoot_des[4];

  Vec3<float> Fr_des[4];

  Vec4<float> contact_state;

  const ConvexMPCDebugSnapshot& debugSnapshot() const {
    return debug_snapshot;
  }
  void resetGaitPhase();

private:
  void _SetupCommand(ControlFSMData<float> & data);

  float _yaw_turn_rate;
  float _yaw_des;

  float _roll_des;
  float _pitch_des;

  float _x_vel_des = 0.;
  float _y_vel_des = 0.;

  // High speed running
  //float _body_height = 0.34;
  float _body_height = 0.29;

  float _body_height_running = 0.29;
  float _body_height_jumping = 0.36;

  void recompute_timing(int iterations_per_mpc);
  void updateMPCIfNeeded(int* mpcTable, ControlFSMData<float>& data, bool omniMode);
  void solveDenseMPC(int *mpcTable, ControlFSMData<float> &data);
  void solveSparseMPC(int *mpcTable, ControlFSMData<float> &data);
  void initSparseMPC(double mass = 9.0);
  int iterationsBetweenMPC;
  int horizonLength;
  int default_iterations_between_mpc;
  float dt;
  float dtMPC;
  int iterationCounter = 0;
  Vec3<float> f_ff[4];
  Vec4<float> swingTimes;
  FootSwingTrajectory<float> footSwingTrajectories[4];
  OffsetDurationGait trotting, legbotTrotting, bounding, pronking, jumping,
      galloping, standing, trotRunning, walking, walking2, pacing;
  MixedFrequncyGait random, random2;
  Mat3<float> Kp, Kd, Kp_stance, Kd_stance;
  bool firstRun = true;
  bool firstSwing[4];
  float swingTimeRemaining[4];
  bool _legbotFixedDiagonalGait0Active = false;
  int _legbotFixedDiagonalGait0StartIteration = 0;
  float stand_traj[6];
  int current_gait = -1;
  int gaitNumber = 4;

  Vec3<float> world_position_desired;
  Vec3<float> rpy_int;
  Vec3<float> rpy_comp;
  float x_comp_integral = 0;
  Vec3<float> pFoot[4];
  CMPC_Result<float> result;
  float trajAll[12*36];
  ConvexMPCDebugSnapshot debug_snapshot;

  MIT_UserParameters* _parameters = nullptr;
  CMPC_Jump jump_state;

  vectorAligned<Vec12<double>> _sparseTrajectory;

  SparseCMPC _sparseCMPC;
  double _sparseMpcMass = -1.;

};


#endif //CHEETAH_SOFTWARE_CONVEXMPCLOCOMOTION_H

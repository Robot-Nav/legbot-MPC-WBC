#include <iostream>
#include <cmath>
#include <Utilities/Timer.h>
#include <Utilities/Utilities_print.h>

#include "ConvexMPCLocomotion.h"
#include "convexMPC_interface.h"
#include "../../../../common/FootstepPlanner/GraphSearch.h"

#include "Gait.h"

//#define DRAW_DEBUG_SWINGS
//#define DRAW_DEBUG_PATH

#ifndef LEGBOT_MPC_DEBUG
#define LEGBOT_MPC_DEBUG 0
#endif

namespace {

constexpr float kLegBotNominalStandingHeight = 0.260f;
constexpr float kLegBotStandingHeightMin = 0.260f;
constexpr float kLegBotStandingHeightMax = 0.295f;
constexpr int kLegBotSlowTrotSegments = 26;
constexpr int kLegBotSlowTrotHalfCycle = kLegBotSlowTrotSegments / 2;
constexpr float kLegBotFixedDiagonalSwingSeconds = 1.20f;
constexpr float kLegBotFixedDiagonalHoldPhase = 0.80f;
constexpr float kLegBotMaxPitchBiasRad = 0.35f;

float ClampFloat(float value, float min_value, float max_value) {
  return fminf(fmaxf(value, min_value), max_value);
}

float LegBotStandingHeightTarget() {
  return ClampFloat(kLegBotNominalStandingHeight, kLegBotStandingHeightMin,
                    kLegBotStandingHeightMax);
}

}  // namespace


////////////////////
// Controller
////////////////////

ConvexMPCLocomotion::ConvexMPCLocomotion(float _dt, int _iterations_between_mpc, MIT_UserParameters* parameters) :
  iterationsBetweenMPC(_iterations_between_mpc),
  horizonLength(10),
  dt(_dt),
  trotting(horizonLength, Vec4<int>(0,5,5,0), Vec4<int>(5,5,5,5),"Trotting"),
  legbotTrotting(kLegBotSlowTrotSegments,
               Vec4<int>(0, kLegBotSlowTrotHalfCycle,
                         kLegBotSlowTrotHalfCycle, 0),
               Vec4<int>(kLegBotSlowTrotHalfCycle, kLegBotSlowTrotHalfCycle,
                         kLegBotSlowTrotHalfCycle, kLegBotSlowTrotHalfCycle),
               "LegBot Slow Trotting"),
  bounding(horizonLength, Vec4<int>(5,5,0,0),Vec4<int>(4,4,4,4),"Bounding"),
  //bounding(horizonLength, Vec4<int>(5,5,0,0),Vec4<int>(3,3,3,3),"Bounding"),
  pronking(horizonLength, Vec4<int>(0,0,0,0),Vec4<int>(4,4,4,4),"Pronking"),
  jumping(horizonLength, Vec4<int>(0,0,0,0), Vec4<int>(2,2,2,2), "Jumping"),
  //galloping(horizonLength, Vec4<int>(0,2,7,9),Vec4<int>(6,6,6,6),"Galloping"),
  //galloping(horizonLength, Vec4<int>(0,2,7,9),Vec4<int>(3,3,3,3),"Galloping"),
  galloping(horizonLength, Vec4<int>(0,2,7,9),Vec4<int>(4,4,4,4),"Galloping"),
  standing(horizonLength, Vec4<int>(0,0,0,0),Vec4<int>(10,10,10,10),"Standing"),
  //trotRunning(horizonLength, Vec4<int>(0,5,5,0),Vec4<int>(3,3,3,3),"Trot Running"),
  trotRunning(horizonLength, Vec4<int>(0,5,5,0),Vec4<int>(4,4,4,4),"Trot Running"),
  walking(horizonLength, Vec4<int>(0,3,5,8), Vec4<int>(5,5,5,5), "Walking"),
  walking2(horizonLength, Vec4<int>(0,5,5,0), Vec4<int>(7,7,7,7), "Walking2"),
  pacing(horizonLength, Vec4<int>(5,0,5,0),Vec4<int>(5,5,5,5),"Pacing"),
  random(horizonLength, Vec4<int>(9,13,13,9), 0.4, "Flying nine thirteenths trot"),
  random2(horizonLength, Vec4<int>(8,16,16,8), 0.5, "Double Trot")
{
  _parameters = parameters;
  dtMPC = dt * iterationsBetweenMPC;
  default_iterations_between_mpc = iterationsBetweenMPC;
  printf("[Convex MPC] dt: %.3f iterations: %d, dtMPC: %.3f\n", dt, iterationsBetweenMPC, dtMPC);
  setup_problem(dtMPC, horizonLength, 0.4, 120);
  //setup_problem(dtMPC, horizonLength, 0.4, 650); // DH
  rpy_comp[0] = 0;
  rpy_comp[1] = 0;
  rpy_comp[2] = 0;
  rpy_int[0] = 0;
  rpy_int[1] = 0;
  rpy_int[2] = 0;

  for(int i = 0; i < 4; i++)
    firstSwing[i] = true;

  initSparseMPC();

   pBody_des.setZero();
   vBody_des.setZero();
   aBody_des.setZero();
}

void ConvexMPCLocomotion::initialize(){
  for(int i = 0; i < 4; i++) firstSwing[i] = true;
  firstRun = true;
}

void ConvexMPCLocomotion::initializeStandingTargetForLegBot(
    ControlFSMData<float>& data) {
  auto& seResult = data._stateEstimator->getResult();

  stand_traj[0] = seResult.position[0];
  stand_traj[1] = seResult.position[1];
  stand_traj[2] = LegBotStandingHeightTarget();
  stand_traj[3] = 0.f;
  stand_traj[4] = 0.f;
  stand_traj[5] = seResult.rpy[2];

  world_position_desired[0] = stand_traj[0];
  world_position_desired[1] = stand_traj[1];
  world_position_desired[2] = stand_traj[5];

  _body_height = stand_traj[2];
  _x_vel_des = 0.f;
  _y_vel_des = 0.f;
  _yaw_turn_rate = 0.f;
  _yaw_des = stand_traj[5];
  _roll_des = 0.f;
  _pitch_des = 0.f;

  for(int leg = 0; leg < 4; ++leg) {
    const Vec3<float> currentFootPosition = seResult.position +
      seResult.rBody.transpose() * (data._quadruped->getHipLocation(leg) +
          data._legController->datas[leg].p);
    pFoot[leg] = currentFootPosition;
    pFoot_des[leg] = currentFootPosition;
    vFoot_des[leg].setZero();
    aFoot_des[leg].setZero();
    footSwingTrajectories[leg].setInitialPosition(currentFootPosition);
    footSwingTrajectories[leg].setFinalPosition(currentFootPosition);
  }
}

void ConvexMPCLocomotion::resetGaitPhase(){
  iterationCounter = 0;
  for(int i = 0; i < 4; i++) {
    firstSwing[i] = true;
    swingTimeRemaining[i] = 0.f;
  }
}

void ConvexMPCLocomotion::recompute_timing(int iterations_per_mpc) {
  iterationsBetweenMPC = iterations_per_mpc;
  dtMPC = dt * iterations_per_mpc;
}

void ConvexMPCLocomotion::_SetupCommand(ControlFSMData<float> & data){
  if(data._quadruped->_robotType == RobotType::MINI_CHEETAH){
    _body_height = 0.29;
  }else if(data._quadruped->_robotType == RobotType::LEGBOT){
    _body_height = LegBotStandingHeightTarget();
  }else if(data._quadruped->_robotType == RobotType::CHEETAH_3){
    _body_height = 0.45;
  }else{
    assert(false);
  }

  float x_vel_cmd, y_vel_cmd;
  float filter(0.1);
  if(data.controlParameters->use_rc){
    const rc_control_settings* rc_cmd = data._desiredStateCommand->rcCommand;
    if (data._quadruped->_robotType != RobotType::LEGBOT) {
      data.userParameters->cmpc_gait = rc_cmd->variable[0];
    }
    _yaw_turn_rate = -rc_cmd->omega_des[2];
    x_vel_cmd = rc_cmd->v_des[0];
    y_vel_cmd = rc_cmd->v_des[1] * 0.5;
    _body_height += rc_cmd->height_variation * 0.08;
  }else{
    _yaw_turn_rate = data._desiredStateCommand->rightAnalogStick[0];
    x_vel_cmd = data._desiredStateCommand->leftAnalogStick[1];
    y_vel_cmd = data._desiredStateCommand->leftAnalogStick[0];
  }
  const bool legbot_robot = data._quadruped->_robotType == RobotType::LEGBOT;
  int requested_gait = data.userParameters->cmpc_gait;
  if(requested_gait >= 10) requested_gait -= 10;
  const bool legbot_stationary_trot =
      data._quadruped->_robotType == RobotType::LEGBOT && requested_gait == 0;
  if(legbot_stationary_trot) {
    x_vel_cmd = 0.f;
    y_vel_cmd = 0.f;
    _yaw_turn_rate = 0.f;
    _x_vel_des = 0.f;
    _y_vel_des = 0.f;
  } else {
    _x_vel_des = _x_vel_des*(1-filter) + x_vel_cmd*filter;
    _y_vel_des = _y_vel_des*(1-filter) + y_vel_cmd*filter;
  }

  _yaw_des = data._stateEstimator->getResult().rpy[2] + dt * _yaw_turn_rate;
  _roll_des = 0.;
  _pitch_des =
      (legbot_robot && _parameters)
          ? ClampFloat(static_cast<float>(_parameters->legbot_body_pitch_bias_rad),
                       -kLegBotMaxPitchBiasRad, kLegBotMaxPitchBiasRad)
          : 0.f;

}

template<>
void ConvexMPCLocomotion::run(ControlFSMData<float>& data) {
  bool omniMode = false;

  // Command Setup
  _SetupCommand(data);
  const int rawGaitNumber = data.userParameters->cmpc_gait;
  const bool legbotFixedDiagonalGait0 =
      data._quadruped->_robotType == RobotType::LEGBOT && rawGaitNumber == 10;
  gaitNumber = rawGaitNumber;
  if(gaitNumber >= 10) {
    gaitNumber -= 10;
    omniMode = true;
  }
  if(legbotFixedDiagonalGait0 && !_legbotFixedDiagonalGait0Active) {
    _legbotFixedDiagonalGait0Active = true;
    _legbotFixedDiagonalGait0StartIteration = iterationCounter;
    for(int i = 0; i < 4; ++i) {
      firstSwing[i] = true;
      swingTimeRemaining[i] = 0.f;
    }
    printf("[LEGBOT-FIXED-GAIT0] mode=diag_swing_hold "
           "contact=[1 0 0 1] swing=[0 1 1 0] "
           "swing_seconds=%.3f hold_phase=%.3f\n",
           kLegBotFixedDiagonalSwingSeconds, kLegBotFixedDiagonalHoldPhase);
  } else if(!legbotFixedDiagonalGait0) {
    _legbotFixedDiagonalGait0Active = false;
  }

  auto& seResult = data._stateEstimator->getResult();
  const double mpcMass =
      data._quadruped->_mpcMass > 0. ? data._quadruped->_mpcMass
                                      : data._quadruped->_bodyMass;
  if(std::abs(_sparseMpcMass - mpcMass) > 1e-6) {
    initSparseMPC(mpcMass);
  }
  if(data._quadruped->_robotType == RobotType::LEGBOT) {
    static bool printedLegBotMassDebug = false;
    if(!printedLegBotMassDebug) {
      const double trunkMass = data._quadruped->_bodyMass;
      const double totalMass = data._quadruped->_totalMass;
      const double abadMass = data._quadruped->_abadInertia.getMass();
      const double thighMass = data._quadruped->_hipInertia.getMass();
      const double calfMass = data._quadruped->_kneeInertia.getMass();
      const double legMass = (totalMass - trunkMass) / 4.;
      printf("[MODEL_MASS_DEBUG] trunk_mass=%.6f abad_mass=%.6f "
             "thigh_mass=%.6f calf_mass=%.6f leg_total_mass=%.6f "
             "total_robot_mass=%.6f mpc_mass=%.6f floating_base_body_mass=%.6f\n",
             trunkMass, abadMass, thighMass, calfMass, legMass, totalMass,
             mpcMass, trunkMass);
      printedLegBotMassDebug = true;
    }
  }

  // Check if transition to standing
  if(((gaitNumber == 4) && current_gait != 4) || firstRun)
  {
    stand_traj[0] = seResult.position[0];
    stand_traj[1] = seResult.position[1];
    stand_traj[2] = data._quadruped->_robotType == RobotType::LEGBOT
                        ? LegBotStandingHeightTarget()
                        : _body_height;
    stand_traj[3] = 0;
    stand_traj[4] = 0;
    stand_traj[5] = seResult.rpy[2];
    world_position_desired[0] = stand_traj[0];
    world_position_desired[1] = stand_traj[1];
  }

  // pick gait
  Gait* gait = &trotting;
  if(gaitNumber == 1)
    gait = &bounding;
  else if(gaitNumber == 2)
    gait = &pronking;
  else if(gaitNumber == 3)
    gait = &random;
  else if(gaitNumber == 4)
    gait = &standing;
  else if(gaitNumber == 5)
    gait = &trotRunning;
  else if(gaitNumber == 6)
    gait = &random2;
  else if(gaitNumber == 7)
    gait = &random2;
  else if(gaitNumber == 8)
    gait = &pacing;
  if(data._quadruped->_robotType == RobotType::LEGBOT && gaitNumber == 0)
    gait = &legbotTrotting;
  current_gait = gaitNumber;

  gait->setIterations(iterationsBetweenMPC, iterationCounter);
  jumping.setIterations(iterationsBetweenMPC, iterationCounter);


  jumping.setIterations(27/2, iterationCounter);

  //printf("[%d] [%d]\n", jumping.get_current_gait_phase(), gait->get_current_gait_phase());
  // check jump trigger
  jump_state.trigger_pressed(jump_state.should_jump(jumping.getCurrentGaitPhase()),
      data._desiredStateCommand->trigger_pressed);


  // bool too_high = seResult.position[2] > 0.29;
  // check jump action
  if(jump_state.should_jump(jumping.getCurrentGaitPhase())) {
    gait = &jumping;
    recompute_timing(27/2);
    _body_height = _body_height_jumping;
    currently_jumping = true;

  } else {
    recompute_timing(default_iterations_between_mpc);
    currently_jumping = false;
  }

  if(_body_height < 0.02) {
    _body_height = 0.29;
  }

  // integrate position setpoint
  Vec3<float> v_des_robot(_x_vel_des, _y_vel_des, 0);
  Vec3<float> v_des_world = 
    omniMode ? v_des_robot : seResult.rBody.transpose() * v_des_robot;
  Vec3<float> v_robot = seResult.vWorld;

  //pretty_print(v_des_world, std::cout, "v des world");

  //Integral-esque pitche and roll compensation
  if(fabs(v_robot[0]) > .2)   //avoid dividing by zero
  {
    rpy_int[1] += dt*(_pitch_des - seResult.rpy[1])/v_robot[0];
  }
  if(fabs(v_robot[1]) > 0.1)
  {
    rpy_int[0] += dt*(_roll_des - seResult.rpy[0])/v_robot[1];
  }

  rpy_int[0] = fminf(fmaxf(rpy_int[0], -.25), .25);
  rpy_int[1] = fminf(fmaxf(rpy_int[1], -.25), .25);
  rpy_comp[1] = v_robot[0] * rpy_int[1];
  rpy_comp[0] = v_robot[1] * rpy_int[0] * (gaitNumber!=8);  //turn off for pronking


  for(int i = 0; i < 4; i++) {
    pFoot[i] = seResult.position + 
      seResult.rBody.transpose() * (data._quadruped->getHipLocation(i) + 
          data._legController->datas[i].p);
  }

  if(gait != &standing) {
    world_position_desired += dt * Vec3<float>(v_des_world[0], v_des_world[1], 0);
  }

  // some first time initialization
  if(firstRun)
  {
    world_position_desired[0] = seResult.position[0];
    world_position_desired[1] = seResult.position[1];
    world_position_desired[2] = seResult.rpy[2];

    for(int i = 0; i < 4; i++)
    {

      footSwingTrajectories[i].setHeight(0.05);
      footSwingTrajectories[i].setInitialPosition(pFoot[i]);
      footSwingTrajectories[i].setFinalPosition(pFoot[i]);

    }
    firstRun = false;
  }

  // foot placement
  for(int l = 0; l < 4; l++)
    swingTimes[l] = gait->getCurrentSwingTime(dtMPC, l);

  float side_sign[4] = {-1, 1, -1, 1};
  float interleave_y[4] = {-0.08, 0.08, 0.02, -0.02};
  //float interleave_gain = -0.13;
  float interleave_gain = -0.2;
  //float v_abs = std::fabs(seResult.vBody[0]);
  float v_abs = std::fabs(v_des_robot[0]);
  Vec3<float> plannedFootFinal[4];
  for(int i = 0; i < 4; i++)
  {
    if(firstSwing[i]) {
      swingTimeRemaining[i] = swingTimes[i];
    } else {
      swingTimeRemaining[i] -= dt;
    }
    //if(firstSwing[i]) {
    //footSwingTrajectories[i].setHeight(.05);
    const float swingHeight =
        (data._quadruped->_robotType == RobotType::LEGBOT)
            ? (gaitNumber == 0 ? 0.03f : 0.04f)
            : 0.06f;
    footSwingTrajectories[i].setHeight(swingHeight);
    Vec3<float> offset(0, side_sign[i] * .065, 0);

    Vec3<float> pRobotFrame = (data._quadruped->getHipLocation(i) + offset);

    pRobotFrame[1] += interleave_y[i] * v_abs * interleave_gain;
    float stance_time = gait->getCurrentStanceTime(dtMPC, i);
    Vec3<float> pYawCorrected = 
      coordinateRotation(CoordinateAxis::Z, -_yaw_turn_rate* stance_time / 2) * pRobotFrame;


    Vec3<float> des_vel;
    des_vel[0] = _x_vel_des;
    des_vel[1] = _y_vel_des;
    des_vel[2] = 0.0;

    Vec3<float> Pf = seResult.position + seResult.rBody.transpose() *
          (pYawCorrected + des_vel * swingTimeRemaining[i]);

    //+ seResult.vWorld * swingTimeRemaining[i];

    //float p_rel_max = 0.35f;
    float p_rel_max = 0.3f;

    // Using the estimated velocity is correct
    //Vec3<float> des_vel_world = seResult.rBody.transpose() * des_vel;
    float pfx_rel = seResult.vWorld[0] * (.5 + _parameters->cmpc_bonus_swing) * stance_time +
      .03f*(seResult.vWorld[0]-v_des_world[0]) +
      (0.5f*seResult.position[2]/9.81f) * (seResult.vWorld[1]*_yaw_turn_rate);

    float pfy_rel = seResult.vWorld[1] * .5 * stance_time * dtMPC +
      .03f*(seResult.vWorld[1]-v_des_world[1]) +
      (0.5f*seResult.position[2]/9.81f) * (-seResult.vWorld[0]*_yaw_turn_rate);
    pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
    pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);
    Pf[0] +=  pfx_rel;
    Pf[1] +=  pfy_rel;
    Pf[2] = -0.003;
    //Pf[2] = 0.0;
    plannedFootFinal[i] = Pf;
    if(!firstSwing[i]) {
      footSwingTrajectories[i].setFinalPosition(Pf);
    }

  }

  // calc gait
  iterationCounter++;

  // load LCM leg swing gains
  Kp << 700, 0, 0,
     0, 700, 0,
     0, 0, 150;
  Kp_stance = 0*Kp;


  Kd << 7, 0, 0,
     0, 7, 0,
     0, 0, 7;
  Kd_stance = Kd;
  // gait
  Vec4<float> contactStates = gait->getContactState();
  Vec4<float> swingStates = gait->getSwingState();
  int* mpcTable = gait->getMpcTable();
  if(legbotFixedDiagonalGait0) {
    const float fixedElapsed =
        fmaxf(0.f, (iterationCounter - _legbotFixedDiagonalGait0StartIteration) *
                       dt);
    const float fixedSwingPhase =
        fminf(fixedElapsed / kLegBotFixedDiagonalSwingSeconds,
              kLegBotFixedDiagonalHoldPhase);
    contactStates << 1.f, 0.f, 0.f, 1.f;
    swingStates << 0.f, fixedSwingPhase, fixedSwingPhase, 0.f;
    for(int i = 0; i < horizonLength; ++i) {
      mpcTable[i * 4 + 0] = 1;
      mpcTable[i * 4 + 1] = 0;
      mpcTable[i * 4 + 2] = 0;
      mpcTable[i * 4 + 3] = 1;
    }
    swingTimes[1] = kLegBotFixedDiagonalSwingSeconds;
    swingTimes[2] = kLegBotFixedDiagonalSwingSeconds;
  }
  if(gait == &standing) {
    contactStates = Vec4<float>::Constant(0.5f);
    swingStates.setZero();
    for(int foot = 0; foot < 4; foot++) {
      pFoot_des[foot] = pFoot[foot];
      vFoot_des[foot].setZero();
      aFoot_des[foot].setZero();
    }
  }
  updateMPCIfNeeded(mpcTable, data, omniMode);

  //  StateEstimator* se = hw_i->state_estimator;
  Vec4<float> se_contactState(0,0,0,0);

#ifdef DRAW_DEBUG_PATH
  auto* trajectoryDebug = data.visualizationData->addPath();
  if(trajectoryDebug) {
    trajectoryDebug->num_points = 10;
    trajectoryDebug->color = {0.2, 0.2, 0.7, 0.5};
    for(int i = 0; i < 10; i++) {
      trajectoryDebug->position[i][0] = trajAll[12*i + 3];
      trajectoryDebug->position[i][1] = trajAll[12*i + 4];
      trajectoryDebug->position[i][2] = trajAll[12*i + 5];
      auto* ball = data.visualizationData->addSphere();
      ball->radius = 0.01;
      ball->position = trajectoryDebug->position[i];
      ball->color = {1.0, 0.2, 0.2, 0.5};
    }
  }
#endif

  for(int foot = 0; foot < 4; foot++)
  {
    float contactState = contactStates[foot];
    float swingState = swingStates[foot];
    if(swingState > 0) // foot is in swing
    {
      const bool firstSwingBefore = firstSwing[foot];
      const Vec3<float> oldSwingP0 =
          footSwingTrajectories[foot].getInitialPosition();
      const Vec3<float> oldSwingPf =
          footSwingTrajectories[foot].getFinalPosition();
      float trajectoryPhase = swingState;
      if(firstSwing[foot])
      {
        footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);
        footSwingTrajectories[foot].setFinalPosition(plannedFootFinal[foot]);
        trajectoryPhase = 0.f;
        firstSwing[foot] = false;
      }

#ifdef DRAW_DEBUG_SWINGS
      auto* debugPath = data.visualizationData->addPath();
      if(debugPath) {
        debugPath->num_points = 100;
        debugPath->color = {0.2,1,0.2,0.5};
        float step = (1.f - swingState) / 100.f;
        for(int i = 0; i < 100; i++) {
          footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState + i * step, swingTimes[foot]);
          debugPath->position[i] = footSwingTrajectories[foot].getPosition();
        }
      }
      auto* finalSphere = data.visualizationData->addSphere();
      if(finalSphere) {
        finalSphere->position = footSwingTrajectories[foot].getPosition();
        finalSphere->radius = 0.02;
        finalSphere->color = {0.6, 0.6, 0.2, 0.7};
      }
      footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);
      auto* actualSphere = data.visualizationData->addSphere();
      auto* goalSphere = data.visualizationData->addSphere();
      goalSphere->position = footSwingTrajectories[foot].getPosition();
      actualSphere->position = pFoot[foot];
      goalSphere->radius = 0.02;
      actualSphere->radius = 0.02;
      goalSphere->color = {0.2, 1, 0.2, 0.7};
      actualSphere->color = {0.8, 0.2, 0.2, 0.7};
#endif
      footSwingTrajectories[foot].computeSwingTrajectoryBezier(trajectoryPhase, swingTimes[foot]);


      //      footSwingTrajectories[foot]->updateFF(hw_i->leg_controller->leg_datas[foot].q,
      //                                          hw_i->leg_controller->leg_datas[foot].qd, 0); // velocity dependent friction compensation todo removed
      //hw_i->leg_controller->leg_datas[foot].qd, fsm->main_control_settings.variable[2]);

      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
      Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
      Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) 
        - data._quadruped->getHipLocation(foot);
      Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);

      // Update for WBC
      pFoot_des[foot] = pDesFootWorld;
      vFoot_des[foot] = vDesFootWorld;
      aFoot_des[foot] = footSwingTrajectories[foot].getAcceleration();
      
      if(!data.userParameters->use_wbc){
        // Update leg control command regardless of the usage of WBIC
        data._legController->commands[foot].pDes = pDesLeg;
        data._legController->commands[foot].vDes = vDesLeg;
        data._legController->commands[foot].kpCartesian = Kp;
        data._legController->commands[foot].kdCartesian = Kd;
      }
      if(firstSwingBefore && data._quadruped->_robotType == RobotType::LEGBOT &&
         gaitNumber == 0) {
        const auto& cmd = data._legController->commands[foot];
        printf("[GAIT0-FIRSTSWING-DIAG] leg=%d phase=%d swing_phase=%.5f "
               "trajectory_phase=%.5f firstSwing_before=1 firstSwing_after=%d "
               "current_foot_p=[%.5f %.5f %.5f] "
               "old_swing_p0=[%.5f %.5f %.5f] old_swing_pf=[%.5f %.5f %.5f] "
               "new_swing_p0=[%.5f %.5f %.5f] new_swing_pf=[%.5f %.5f %.5f] "
               "generated_pDes=[%.5f %.5f %.5f] generated_vDes=[%.5f %.5f %.5f] "
               "qDes=[%.5f %.5f %.5f] raw_q_step=unavailable_in_cmpc "
               "tauFF=[%.5f %.5f %.5f]\n",
               foot, gait->getCurrentGaitPhase(), swingState, trajectoryPhase,
               firstSwing[foot] ? 1 : 0, pFoot[foot][0], pFoot[foot][1],
               pFoot[foot][2], oldSwingP0[0], oldSwingP0[1], oldSwingP0[2],
               oldSwingPf[0], oldSwingPf[1], oldSwingPf[2],
               footSwingTrajectories[foot].getInitialPosition()[0],
               footSwingTrajectories[foot].getInitialPosition()[1],
               footSwingTrajectories[foot].getInitialPosition()[2],
               footSwingTrajectories[foot].getFinalPosition()[0],
               footSwingTrajectories[foot].getFinalPosition()[1],
               footSwingTrajectories[foot].getFinalPosition()[2],
               pDesFootWorld[0], pDesFootWorld[1], pDesFootWorld[2],
               vDesFootWorld[0], vDesFootWorld[1], vDesFootWorld[2],
               cmd.qDes[0], cmd.qDes[1], cmd.qDes[2],
               cmd.tauFeedForward[0], cmd.tauFeedForward[1],
               cmd.tauFeedForward[2]);
      }
    }
    else // foot is in stance
    {
      firstSwing[foot] = true;

#ifdef DRAW_DEBUG_SWINGS
      auto* actualSphere = data.visualizationData->addSphere();
      actualSphere->position = pFoot[foot];
      actualSphere->radius = 0.02;
      actualSphere->color = {0.2, 0.2, 0.8, 0.7};
#endif

      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
      Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
      Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - data._quadruped->getHipLocation(foot);
      Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);
      //cout << "Foot " << foot << " relative velocity desired: " << vDesLeg.transpose() << "\n";

      if(!data.userParameters->use_wbc){
        data._legController->commands[foot].pDes = pDesLeg;
        data._legController->commands[foot].vDes = vDesLeg;
        data._legController->commands[foot].kpCartesian = Kp_stance;
        data._legController->commands[foot].kdCartesian = Kd_stance;

        data._legController->commands[foot].forceFeedForward = f_ff[foot];
        data._legController->commands[foot].kdJoint = Mat3<float>::Identity() * 0.2;

        //      footSwingTrajectories[foot]->updateFF(hw_i->leg_controller->leg_datas[foot].q,
        //                                          hw_i->leg_controller->leg_datas[foot].qd, 0); todo removed
        // hw_i->leg_controller->leg_commands[foot].tau_ff += 0*footSwingController[foot]->getTauFF();
      }else{ // Stance foot damping
        data._legController->commands[foot].pDes = pDesLeg;
        data._legController->commands[foot].vDes = vDesLeg;
        data._legController->commands[foot].kpCartesian = 0.*Kp_stance;
        data._legController->commands[foot].kdCartesian = Kd_stance;
      }
      //            cout << "Foot " << foot << " force: " << f_ff[foot].transpose() << "\n";
      se_contactState[foot] = contactState;

      // Update for WBC
      //Fr_des[foot] = -f_ff[foot];
    }
  }

  // se->set_contact_state(se_contactState); todo removed
  data._stateEstimator->setContactPhase(se_contactState);

  debug_snapshot.valid = true;
  debug_snapshot.gaitNumber = gaitNumber;
  debug_snapshot.currentGait = current_gait;
  debug_snapshot.gaitPhase = gait->getCurrentGaitPhase();
  debug_snapshot.iterationCounter = iterationCounter;
  debug_snapshot.contactStates = contactStates;
  debug_snapshot.swingStates = swingStates;
  debug_snapshot.seContactState = se_contactState;
  debug_snapshot.swingTimes = swingTimes;
  for(int leg = 0; leg < 4; ++leg) {
    debug_snapshot.swingTimeRemaining[leg] = swingTimeRemaining[leg];
    debug_snapshot.pFoot[leg] = pFoot[leg];
    debug_snapshot.pFootDes[leg] = pFoot_des[leg];
    debug_snapshot.vFootDes[leg] = vFoot_des[leg];
    debug_snapshot.swingP0[leg] =
        footSwingTrajectories[leg].getInitialPosition();
    debug_snapshot.swingPf[leg] =
        footSwingTrajectories[leg].getFinalPosition();
    debug_snapshot.FrDes[leg] = Fr_des[leg];
    debug_snapshot.mpcTableNow[leg] = mpcTable[leg];
    debug_snapshot.firstSwing[leg] = firstSwing[leg] ? 1 : 0;
  }

  // Update For WBC
  pBody_des[0] = world_position_desired[0];
  pBody_des[1] = world_position_desired[1];
  pBody_des[2] = gait == &standing ? stand_traj[2] : _body_height;

  vBody_des[0] = v_des_world[0];
  vBody_des[1] = v_des_world[1];
  vBody_des[2] = 0.;

  aBody_des.setZero();

  pBody_RPY_des[0] = _roll_des;
  pBody_RPY_des[1] = _pitch_des;
  pBody_RPY_des[2] = _yaw_des;

  vBody_Ori_des[0] = 0.;
  vBody_Ori_des[1] = 0.;
  vBody_Ori_des[2] = _yaw_turn_rate;

  contact_state = contactStates;
  // END of WBC Update

#if LEGBOT_MPC_DEBUG
  if(data._quadruped->_robotType == RobotType::LEGBOT) {
    static int legbotDebugCounter = 0;
    if((legbotDebugCounter++ % 500) == 0) {
      printf("[LegBot MPC debug] gait=%d body_z=%.3f des_z=%.3f "
             "vWorld=[%.3f %.3f %.3f] vDesWorld=[%.3f %.3f %.3f] "
             "contact=[%.2f %.2f %.2f %.2f]\n",
             gaitNumber, seResult.position[2], _body_height,
             seResult.vWorld[0], seResult.vWorld[1], seResult.vWorld[2],
             v_des_world[0], v_des_world[1], v_des_world[2],
             contact_state[0], contact_state[1], contact_state[2],
             contact_state[3]);
      for(int leg = 0; leg < 4; leg++) {
        printf("  leg %d Fr_des=[% .2f % .2f % .2f] pFoot=[% .3f % .3f % .3f]\n",
               leg, Fr_des[leg][0], Fr_des[leg][1], Fr_des[leg][2],
               pFoot[leg][0], pFoot[leg][1], pFoot[leg][2]);
      }
    }
  }
#endif


}

template<>
void ConvexMPCLocomotion::run(ControlFSMData<double>& data) {
  (void)data;
  printf("call to old CMPC with double!\n");

}

void ConvexMPCLocomotion::updateMPCIfNeeded(int *mpcTable, ControlFSMData<float> &data, bool omniMode) {
  //iterationsBetweenMPC = 30;
  if((iterationCounter % iterationsBetweenMPC) == 0)
  {
    auto seResult = data._stateEstimator->getResult();
    float* p = seResult.position.data();

    Vec3<float> v_des_robot(_x_vel_des, _y_vel_des,0);
    Vec3<float> v_des_world = omniMode ? v_des_robot : seResult.rBody.transpose() * v_des_robot;
    //float trajInitial[12] = {0,0,0, 0,0,.25, 0,0,0,0,0,0};


    //printf("Position error: %.3f, integral %.3f\n", pxy_err[0], x_comp_integral);

    if(current_gait == 4)
    {
      float trajInitial[12] = {
        _roll_des,
        _pitch_des /*-hw_i->state_estimator->se_ground_pitch*/,
        (float)stand_traj[5]/*+(float)stateCommand->data.stateDes[11]*/,
        (float)stand_traj[0]/*+(float)fsm->main_control_settings.p_des[0]*/,
        (float)stand_traj[1]/*+(float)fsm->main_control_settings.p_des[1]*/,
        (float)stand_traj[2]/*fsm->main_control_settings.p_des[2]*/,
        0,0,0,0,0,0};

      for(int i = 0; i < horizonLength; i++)
        for(int j = 0; j < 12; j++)
          trajAll[12*i+j] = trajInitial[j];
    }

    else
    {
      const float max_pos_error = .1;
      float xStart = world_position_desired[0];
      float yStart = world_position_desired[1];

      if(xStart - p[0] > max_pos_error) xStart = p[0] + max_pos_error;
      if(p[0] - xStart > max_pos_error) xStart = p[0] - max_pos_error;

      if(yStart - p[1] > max_pos_error) yStart = p[1] + max_pos_error;
      if(p[1] - yStart > max_pos_error) yStart = p[1] - max_pos_error;

      world_position_desired[0] = xStart;
      world_position_desired[1] = yStart;

      const float roll_cmd = _roll_des + rpy_comp[0];
      const float pitch_cmd = _pitch_des + rpy_comp[1];
      float trajInitial[12] = {roll_cmd,  // 0
        pitch_cmd,    // 1
        _yaw_des,    // 2
        //yawStart,    // 2
        xStart,                                   // 3
        yStart,                                   // 4
        (float)_body_height,      // 5
        0,                                        // 6
        0,                                        // 7
        _yaw_turn_rate,  // 8
        v_des_world[0],                           // 9
        v_des_world[1],                           // 10
        0};                                       // 11

      for(int i = 0; i < horizonLength; i++)
      {
        for(int j = 0; j < 12; j++)
          trajAll[12*i+j] = trajInitial[j];

        if(i == 0) // start at current position  TODO consider not doing this
        {
          //trajAll[3] = hw_i->state_estimator->se_pBody[0];
          //trajAll[4] = hw_i->state_estimator->se_pBody[1];
          trajAll[2] = seResult.rpy[2];
        }
        else
        {
          trajAll[12*i + 3] = trajAll[12 * (i - 1) + 3] + dtMPC * v_des_world[0];
          trajAll[12*i + 4] = trajAll[12 * (i - 1) + 4] + dtMPC * v_des_world[1];
          trajAll[12*i + 2] = trajAll[12 * (i - 1) + 2] + dtMPC * _yaw_turn_rate;
        }
      }
    }
    Timer solveTimer;

    if(_parameters->cmpc_use_sparse > 0.5) {
      solveSparseMPC(mpcTable, data);
    } else {
      solveDenseMPC(mpcTable, data);
    }
    //printf("TOTAL SOLVE TIME: %.3f\n", solveTimer.getMs());
  }

}

void ConvexMPCLocomotion::solveDenseMPC(int *mpcTable, ControlFSMData<float> &data) {
  auto seResult = data._stateEstimator->getResult();

  //float Q[12] = {0.25, 0.25, 10, 2, 2, 20, 0, 0, 0.3, 0.2, 0.2, 0.2};

  float Q[12] = {0.25, 0.25, 10, 2, 2, 50, 0, 0, 0.3, 0.2, 0.2, 0.1};

  //float Q[12] = {0.25, 0.25, 10, 2, 2, 40, 0, 0, 0.3, 0.2, 0.2, 0.2};
  float yaw = seResult.rpy[2];
  float* weights = Q;
  float alpha = 4e-5; // make setting eventually
  //float alpha = 4e-7; // make setting eventually: DH
  float* p = seResult.position.data();
  float* v = seResult.vWorld.data();
  float* w = seResult.omegaWorld.data();
  float* q = seResult.orientation.data();

  float r[12];
  for(int i = 0; i < 12; i++)
    r[i] = pFoot[i%4][i/4]  - seResult.position[i/4];

  //printf("current posistion: %3.f %.3f %.3f\n", p[0], p[1], p[2]);

  if(alpha > 1e-4) {
    std::cout << "Alpha was set too high (" << alpha << ") adjust to 1e-5\n";
    alpha = 1e-5;
  }

  Vec3<float> pxy_act(p[0], p[1], 0);
  Vec3<float> pxy_des(world_position_desired[0], world_position_desired[1], 0);
  //Vec3<float> pxy_err = pxy_act - pxy_des;
  float pz_err = p[2] - _body_height;
  Vec3<float> vxy(seResult.vWorld[0], seResult.vWorld[1], 0);

  Timer t1;
  dtMPC = dt * iterationsBetweenMPC;
  setup_problem(dtMPC,horizonLength,0.4,120);
  //setup_problem(dtMPC,horizonLength,0.4,650); //DH
  update_x_drag(x_comp_integral);
  if(vxy[0] > 0.3 || vxy[0] < -0.3) {
    //x_comp_integral += _parameters->cmpc_x_drag * pxy_err[0] * dtMPC / vxy[0];
    x_comp_integral += _parameters->cmpc_x_drag * pz_err * dtMPC / vxy[0];
  }

  //printf("pz err: %.3f, pz int: %.3f\n", pz_err, x_comp_integral);

  update_solver_settings(_parameters->jcqp_max_iter, _parameters->jcqp_rho,
      _parameters->jcqp_sigma, _parameters->jcqp_alpha, _parameters->jcqp_terminate, _parameters->use_jcqp);
  const float mpcMass =
      data._quadruped->_mpcMass > 0.f ? data._quadruped->_mpcMass
                                      : data._quadruped->_bodyMass;
  Mat3<float> bodyInertia = data._quadruped->_bodyInertia.getInertiaTensor();
  float bodyInertiaRowMajor[9];
  for(int row = 0; row < 3; ++row) {
    for(int col = 0; col < 3; ++col) {
      bodyInertiaRowMajor[3 * row + col] = bodyInertia(row, col);
    }
  }
  update_problem_robot_params(mpcMass, bodyInertiaRowMajor);
  //t1.stopPrint("Setup MPC");

  Timer t2;
  //cout << "dtMPC: " << dtMPC << "\n";
  update_problem_data_floats(p,v,q,w,r,yaw,weights,trajAll,alpha,mpcTable);
  //t2.stopPrint("Run MPC");
  //printf("MPC Solve time %f ms\n", t2.getMs());

  for(int leg = 0; leg < 4; leg++)
  {
    Vec3<float> f;
    for(int axis = 0; axis < 3; axis++)
      f[axis] = get_solution(leg*3 + axis);

    //printf("[%d] %7.3f %7.3f %7.3f\n", leg, f[0], f[1], f[2]);

    f_ff[leg] = -seResult.rBody * f;
    // Update for WBC
    Fr_des[leg] = f;
  }
}

void ConvexMPCLocomotion::solveSparseMPC(int *mpcTable, ControlFSMData<float> &data) {
  // X0, contact trajectory, state trajectory, feet, get result!
  (void)mpcTable;
  (void)data;
  auto seResult = data._stateEstimator->getResult();

  std::vector<ContactState> contactStates;
  for(int i = 0; i < horizonLength; i++) {
    contactStates.emplace_back(mpcTable[i*4 + 0], mpcTable[i*4 + 1], mpcTable[i*4 + 2], mpcTable[i*4 + 3]);
  }

  for(int i = 0; i < horizonLength; i++) {
    for(u32 j = 0; j < 12; j++) {
      _sparseTrajectory[i][j] = trajAll[i*12 + j];
    }
  }

  Vec12<float> feet;
  for(u32 foot = 0; foot < 4; foot++) {
    for(u32 axis = 0; axis < 3; axis++) {
      feet[foot*3 + axis] = pFoot[foot][axis] - seResult.position[axis];
    }
  }

  _sparseCMPC.setX0(seResult.position, seResult.vWorld, seResult.orientation, seResult.omegaWorld);
  _sparseCMPC.setContactTrajectory(contactStates.data(), contactStates.size());
  _sparseCMPC.setStateTrajectory(_sparseTrajectory);
  _sparseCMPC.setFeet(feet);
  _sparseCMPC.run();

  Vec12<float> resultForce = _sparseCMPC.getResult();

  for(u32 foot = 0; foot < 4; foot++) {
    Vec3<float> force(resultForce[foot*3], resultForce[foot*3 + 1], resultForce[foot*3 + 2]);
    //printf("[%d] %7.3f %7.3f %7.3f\n", foot, force[0], force[1], force[2]);
    f_ff[foot] = -seResult.rBody * force;
    Fr_des[foot] = force;
  }
}

void ConvexMPCLocomotion::initSparseMPC(double mass) {
  Mat3<double> baseInertia;
  baseInertia << 0.04427, -0.000146, -0.024292,
      -0.000146, 0.105409, -0.000032,
      -0.024292, -0.000032, 0.091875;
  double maxForce = 120;

  std::vector<double> dtTraj;
  for(int i = 0; i < horizonLength; i++) {
    dtTraj.push_back(dtMPC);
  }

  Vec12<double> weights;
  weights << 0.25, 0.25, 10, 2, 2, 20, 0, 0, 0.3, 0.2, 0.2, 0.2;
  //weights << 0,0,0,1,1,10,0,0,0,0.2,0.2,0;

  _sparseCMPC.setRobotParameters(baseInertia, mass, maxForce);
  _sparseMpcMass = mass;
  _sparseCMPC.setFriction(0.4);
  _sparseCMPC.setWeights(weights, 4e-5);
  _sparseCMPC.setDtTrajectory(dtTraj);

  _sparseTrajectory.resize(horizonLength);
}

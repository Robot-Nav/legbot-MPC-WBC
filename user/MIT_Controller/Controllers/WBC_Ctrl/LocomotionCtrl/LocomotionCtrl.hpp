#ifndef LOCOMOTION_CONTROLLER
#define LOCOMOTION_CONTROLLER


#include <WBC_Ctrl/WBC_Ctrl.hpp>

template<typename T>
class LocomotionCtrlData{
  public:
    Vec3<T> pBody_des;
    Vec3<T> vBody_des;
    Vec3<T> aBody_des;
    Vec3<T> pBody_RPY_des;
    Vec3<T> vBody_Ori_des;

    Vec3<T> pFoot_des[4];
    Vec3<T> vFoot_des[4];
    Vec3<T> aFoot_des[4];
    Vec3<T> Fr_des[4];

    Vec4<T> contact_state;
};

template<typename T>
class LocomotionCtrl: public WBC_Ctrl<T>{
  public:
    LocomotionCtrl(FloatingBaseModel<T> model);
    virtual ~LocomotionCtrl() {}

  protected:
    virtual void _ContactTaskUpdate(
        void * input, ControlFSMData<T> & data);
    virtual void _PostComputeCommand(ControlFSMData<T> & data) override;
    void _ParameterSetup(const MIT_UserParameters* param);
    virtual void _LCM_PublishData();

    LocomotionCtrlData<T>* _input_data = nullptr;

    Vec3<T> pre_foot_vel[4];

    Vec3<T> _Fr_result[4];
    T _body_mass = T(0);
    T _trunk_mass = T(0);
    T _total_robot_mass = T(0);
    T _mpc_mass = T(0);
    Quadruped<T>* _quadruped = nullptr;
    Quat<T> _quat_des;
    bool _has_prev_contact_state = false;
    Vec4<T> _prev_contact_state = Vec4<T>::Zero();
    Vec3<T> _last_q_des[4] = {Vec3<T>::Zero(), Vec3<T>::Zero(),
                              Vec3<T>::Zero(), Vec3<T>::Zero()};
    Vec3<T> _last_qd_des[4] = {Vec3<T>::Zero(), Vec3<T>::Zero(),
                               Vec3<T>::Zero(), Vec3<T>::Zero()};
    int _touchdown_slew_count[4] = {0, 0, 0, 0};
    // debug 缓存
    T _kp_pos_z = T(0), _kd_pos_z = T(0);
};

#endif

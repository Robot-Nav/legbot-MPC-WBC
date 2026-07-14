#ifndef WBC_CONTROLLER_H
#define WBC_CONTROLLER_H

#include <FSM_States/ControlFSMData.h>
#include <Dynamics/FloatingBaseModel.h>
#include <Dynamics/Quadruped.h>
#include "cppTypes.h"
#include <WBC/QPWBC.hpp>

#include <lcm-cpp.hpp>
#include "wbc_test_data_t.hpp"

#define WBCtrl WBC_Ctrl<T>

class MIT_UserParameters;

template<typename T>
class WBC_Ctrl{
  public:
    WBC_Ctrl(FloatingBaseModel<T> model);
    virtual ~WBC_Ctrl();

    void run(void * input, ControlFSMData<T> & data);

  protected:
    virtual void _ContactTaskUpdate(void * input, ControlFSMData<T> & data) = 0;
    virtual void _LCM_PublishData(){}
    void _UpdateModel(const StateEstimate<T> & state_est, const LegControllerData<T> * leg_data);
    void _UpdateLegCMD(ControlFSMData<T> & data);
    void _ComputeWBC(T dt);
    virtual void _PostComputeCommand(ControlFSMData<T> & data) {
      (void)data;
    }

    // 统一 QP-WBC（非模板，仅 double；T 在边界 cast）
    QPWBC* _qp_wbc;

    // FloatingBaseModel 保留（供 _LCM_PublishData 读 _pGC/_vGC、结构、hip 位置）
    FloatingBaseModel<T> _model;

    DMat<T> _A;
    DMat<T> _Ainv;
    DVec<T> _grav;
    DVec<T> _coriolis;

    FBModelState<T> _state;

    DVec<T> _full_config;
    DVec<T> _tau_ff;
    DVec<T> _des_jpos;
    DVec<T> _des_jvel;

    std::vector<T> _Kp_joint, _Kd_joint;

    unsigned long long _iter;

    lcm::LCM _wbcLCM;
    wbc_test_data_t _wbc_data_lcm;
};
#endif

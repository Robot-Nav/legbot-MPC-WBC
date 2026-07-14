/*!
 * @file JPosInitializer.h
 * @brief Controller to initialize the position of the legs on power-on
 */

#ifndef JPOS_INITIALIZER
#define JPOS_INITIALIZER

#include "Controllers/LegController.h"
#include "Dynamics/Quadruped.h"
#include "Utilities/BSplineBasic.h"
#include <string>

/*!
 * Controller to initialize the position of the legs on power-on
 */
template <typename T>
class JPosInitializer {
 public:
  JPosInitializer(T end_time, float dt);
  JPosInitializer(T end_time, float dt, const std::string& parameter_file);
  ~JPosInitializer();

  bool IsInitialized(LegController<T>*);

 private:
  void _UpdateParam();
  void _UpdateParam(const std::string& parameter_file);
  void _UpdateInitial(const LegController<T>* ctrl);
  bool _b_first_visit;
  T _end_time;
  T _curr_time;
  T _dt;

  std::vector<T> _ini_jpos;
  std::vector<T> _target_jpos;
  std::vector<T> _mid_jpos;

  BS_Basic<T, cheetah::num_act_joint, 3, 1, 2, 2> _jpos_trj;
};
#endif

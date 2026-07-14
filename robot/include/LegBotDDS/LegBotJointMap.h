#ifndef PROJECT_LEGBOT_JOINT_MAP_H
#define PROJECT_LEGBOT_JOINT_MAP_H

#include <array>
#include <cstddef>
#include <stdexcept>

namespace legbot {

constexpr std::size_t kNumLegs = 4;
constexpr std::size_t kNumJointsPerLeg = 3;
constexpr std::size_t kNumJoints = kNumLegs * kNumJointsPerLeg;

enum class ModelLeg : std::size_t { FR = 0, FL = 1, RR = 2, RL = 3 };
enum class ModelJoint : std::size_t { Hip = 0, Thigh = 1, Calf = 2 };

inline const std::array<int, kNumJoints>& ModelToDdsMap() {
  // Cheetah model order: FR, FL, RR, RL. DDS order currently matches it.
  static const std::array<int, kNumJoints> map = {
      0, 1, 2,    // FR hip/thigh/calf
      3, 4, 5,    // FL hip/thigh/calf
      6, 7, 8,    // RR hip/thigh/calf
      9, 10, 11,  // RL hip/thigh/calf
  };
  return map;
}

inline const std::array<double, kNumJoints>& JointSigns() {
  // Keep this explicit even though the first hardware bring-up uses +1 for all
  // joints. q, dq and tau must share the same sign convention.
  static const std::array<double, kNumJoints> signs = {
      1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1.,
  };
  return signs;
}

inline int ModelToDdsJoint(int model_joint) {
  if (model_joint < 0 || model_joint >= static_cast<int>(kNumJoints)) {
    throw std::out_of_range("LegBot model joint index out of range");
  }
  return ModelToDdsMap()[static_cast<std::size_t>(model_joint)];
}

inline int DdsToModelJoint(int dds_joint) {
  if (dds_joint < 0 || dds_joint >= static_cast<int>(kNumJoints)) {
    throw std::out_of_range("LegBot DDS joint index out of range");
  }
  const auto& map = ModelToDdsMap();
  for (std::size_t model = 0; model < map.size(); ++model) {
    if (map[model] == dds_joint) return static_cast<int>(model);
  }
  throw std::out_of_range("LegBot DDS joint index is not mapped");
}

inline int ModelJointIndex(int leg, int joint) {
  if (leg < 0 || leg >= static_cast<int>(kNumLegs) || joint < 0 ||
      joint >= static_cast<int>(kNumJointsPerLeg)) {
    throw std::out_of_range("LegBot leg/joint index out of range");
  }
  return leg * static_cast<int>(kNumJointsPerLeg) + joint;
}

inline int DdsJointIndexFromModel(int leg, int joint) {
  return ModelToDdsJoint(ModelJointIndex(leg, joint));
}

template <typename T>
std::array<T, kNumJoints> DdsQToModelQ(
    const std::array<T, kNumJoints>& dds_q) {
  std::array<T, kNumJoints> model_q{};
  const auto& signs = JointSigns();
  for (std::size_t model = 0; model < kNumJoints; ++model) {
    const std::size_t dds = static_cast<std::size_t>(ModelToDdsJoint(model));
    model_q[model] = static_cast<T>(signs[model]) * dds_q[dds];
  }
  return model_q;
}

template <typename T>
std::array<T, kNumJoints> ModelQToDdsQ(
    const std::array<T, kNumJoints>& model_q) {
  std::array<T, kNumJoints> dds_q{};
  const auto& signs = JointSigns();
  for (std::size_t model = 0; model < kNumJoints; ++model) {
    const std::size_t dds = static_cast<std::size_t>(ModelToDdsJoint(model));
    dds_q[dds] = static_cast<T>(signs[model]) * model_q[model];
  }
  return dds_q;
}

template <typename T>
std::array<T, kNumJoints> DdsDqToModelDq(
    const std::array<T, kNumJoints>& dds_dq) {
  return DdsQToModelQ(dds_dq);
}

template <typename T>
std::array<T, kNumJoints> ModelTauToDdsTau(
    const std::array<T, kNumJoints>& model_tau) {
  return ModelQToDdsQ(model_tau);
}

inline const char* JointNameDds(int dds_joint) {
  static const std::array<const char*, kNumJoints> names = {
      "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
      "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
      "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
      "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
  };
  if (dds_joint < 0 || dds_joint >= static_cast<int>(kNumJoints)) {
    return "unknown_joint";
  }
  return names[static_cast<std::size_t>(dds_joint)];
}

}  // namespace legbot

#endif  // PROJECT_LEGBOT_JOINT_MAP_H

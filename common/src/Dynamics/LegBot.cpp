/*! @file LegBot.cpp
 *  @brief Utility function to build a LegBot Quadruped object
 */

#include "Dynamics/LegBot.h"

/*!
 * Generate a Quadruped model of LegBot.
 *
 * Source model: models/MJCF/legbot/legbot_mpc_scene.xml
 * Nominal default body/COM height used by the control stack: 0.260 m.
 */
template <typename T>
Quadruped<T> buildLegBot() {
  Quadruped<T> legbot;
  legbot._robotType = RobotType::LEGBOT;

  legbot._bodyMass = 6.583964;
  const T abadMass = T(0.079643);
  const T hipMass = T(1.55038);
  const T kneeMass = T(0.183986);
  const T legMass = abadMass + hipMass + kneeMass;
  legbot._totalMass = legbot._bodyMass + T(4) * legMass;
  legbot._mpcMass = legbot._totalMass;
  legbot._bodyLength = 0.50;
  legbot._bodyWidth = 0.205;
  legbot._bodyHeight = 0.138;

  legbot._abadGearRatio = 1.;
  legbot._hipGearRatio = 1.;
  legbot._kneeGearRatio = 1.;
  legbot._abadLinkLength = 0.0975;
  legbot._hipLinkLength = 0.1985;
  legbot._kneeLinkY_offset = 0.;
  legbot._kneeLinkLength = 0.214;
  legbot._maxLegLength = legbot._hipLinkLength + legbot._kneeLinkLength;

  // TODO(legbot): Quadruped currently exposes a single torque limit. The MJCF
  // has hip/thigh actuatorfrcrange +/-17 Nm and calf +/-34 Nm. Keep the
  // conservative existing scalar path for now, then add per-joint limits.
  legbot._motorTauMax = 34.f;
  legbot._batteryV = 24;
  legbot._motorKT = .05;
  legbot._motorR = 0.173;
  legbot._jointDamping = .1;
  legbot._jointDryFriction = .2;

  Mat3<T> bodyRotationalInertia;
  bodyRotationalInertia << 0.04427, -0.000146, -0.024292,
      -0.000146, 0.105409, -0.000032,
      -0.024292, -0.000032, 0.091875;
  Vec3<T> bodyCOM(-0.008, 0.0, 0.0);
  SpatialInertia<T> bodyInertia(legbot._bodyMass, bodyCOM,
                                bodyRotationalInertia);

  Mat3<T> abadRotationalInertia;
  abadRotationalInertia << 8.72171e-05, 0, 0,
      0, 7.7874e-05, 0,
      0, 0, 2.29089e-05;
  Vec3<T> abadCOM(-0.027882, 0.015355, 0);
  SpatialInertia<T> abadInertia(abadMass, abadCOM, abadRotationalInertia);

  Mat3<T> hipRotationalInertia;
  hipRotationalInertia << 0.00679074, 0, 0,
      0, 0.00586921, 0,
      0, 0, 0.00204705;
  Vec3<T> hipCOM(-0.000498, 0.038065, -0.024467);
  SpatialInertia<T> hipInertia(hipMass, hipCOM, hipRotationalInertia);

  Mat3<T> kneeRotationalInertia;
  kneeRotationalInertia << 0.00125035, 0, 0,
      0, 0.0012368, 0,
      0, 0, 3.55485e-05;
  Vec3<T> kneeCOM(0.00261631, 0, -0.125302);
  SpatialInertia<T> kneeInertia(kneeMass, kneeCOM, kneeRotationalInertia);

  // TODO(legbot): MJCF motor rotor inertias are not represented separately.
  // Use zero-mass, tiny-inertia rotors to satisfy the Cheetah model layout.
  Mat3<T> rotorRotationalInertia = Mat3<T>::Identity() * T(1e-6);
  Vec3<T> rotorCOM(0, 0, 0);
  SpatialInertia<T> rotorInertia(0.0, rotorCOM, rotorRotationalInertia);

  legbot._abadInertia = abadInertia;
  legbot._hipInertia = hipInertia;
  legbot._kneeInertia = kneeInertia;
  legbot._abadRotorInertia = rotorInertia;
  legbot._hipRotorInertia = rotorInertia;
  legbot._kneeRotorInertia = rotorInertia;
  legbot._bodyInertia = bodyInertia;

  legbot._abadLocation = Vec3<T>(0.18453, 0.051, 0);
  legbot._abadRotorLocation = legbot._abadLocation;
  legbot._hipLocation = Vec3<T>(0, legbot._abadLinkLength, 0);
  legbot._hipRotorLocation = Vec3<T>(0, legbot._abadLinkLength, 0);
  legbot._kneeLocation = Vec3<T>(0, 0, -legbot._hipLinkLength);
  legbot._kneeRotorLocation = Vec3<T>(0, 0, 0);

  return legbot;
}

template Quadruped<float> buildLegBot<float>();
template Quadruped<double> buildLegBot<double>();

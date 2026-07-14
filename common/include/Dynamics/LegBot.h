/*! @file LegBot.h
 *  @brief Utility function to build a LegBot Quadruped object
 *
 * The parameters in this file are mapped from models/MJCF/legbot/legbot_mpc_scene.xml.
 * This lets the existing Cheetah-Software FloatingBaseModel, LegController,
 * Convex MPC and WBC code reason about the LegBot morphology.
 */

#ifndef PROJECT_LEGBOT_H
#define PROJECT_LEGBOT_H

#include "FloatingBaseModel.h"
#include "Quadruped.h"

template <typename T>
Quadruped<T> buildLegBot();

#endif  // PROJECT_LEGBOT_H

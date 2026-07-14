/*! @file test_legbot_model.cpp
 *  @brief Test the LegBot model used by the Cheetah-Software stack
 */

#include "gtest/gtest.h"

#include "Dynamics/FloatingBaseModel.h"
#include "Dynamics/LegBot.h"

TEST(LegBot, legbotModelBuilds) {
  auto legbot = buildLegBot<double>();
  FloatingBaseModel<double> model = legbot.buildModel();

  EXPECT_EQ(legbot._robotType, RobotType::LEGBOT);
  EXPECT_NEAR(legbot._bodyMass, 6.743964, 1e-6);
  EXPECT_NEAR(legbot._totalMass, 14.0, 1e-5);
  EXPECT_NEAR(legbot._mpcMass, legbot._totalMass, 1e-9);
  EXPECT_NEAR(legbot._abadLocation[0], 0.18453, 1e-6);
  EXPECT_NEAR(legbot._abadLocation[1], 0.051, 1e-6);
  EXPECT_NEAR(legbot._hipLinkLength, 0.1985, 1e-6);
  EXPECT_NEAR(legbot._kneeLinkLength, 0.214, 1e-6);
  EXPECT_NEAR(legbot._maxLegLength, 0.4125, 1e-6);

  EXPECT_NEAR(model.totalNonRotorMass(), 14.0, 1e-5);
}

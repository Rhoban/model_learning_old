#include "rhoban_model_learning/basic_models/pose_2d_model.h"

#include <gtest/gtest.h>

using namespace std;
using namespace rhoban_model_learning;

static double epsilon = std::pow(10, -10);

TEST(defaultConstructor, defaultPose)
{
  Pose2DModel p2d;
  Eigen::VectorXd parameters = p2d.getParameters();
  EXPECT_EQ(parameters.rows(), 3);
  EXPECT_FLOAT_EQ(parameters(0), 0);
  EXPECT_FLOAT_EQ(parameters(1), 0);
  EXPECT_FLOAT_EQ(parameters(2), 0);
}

TEST(get3DPose, defaultPose)
{
  Pose2DModel p2d;
  PoseModel p3d = p2d.get3DPose();
  Eigen::Vector3d pos_world(0, 1, 2);
  Eigen::Vector3d pos_self = p3d.getPosInSelf(pos_world);
  for (int d = 0; d < 3; d++)
  {
    EXPECT_FLOAT_EQ(pos_self(d), pos_world(d));
  }
}

TEST(get3DPose, offsetPose)
{
  Pose2DModel p2d;
  PoseModel p3d = p2d.get3DPose();

  Eigen::VectorXd parameters = Eigen::VectorXd::Zero(7);
  parameters(0) = -1;
  parameters(1) = 1;  // Pose is at (-1,1)
  p3d.setParameters(parameters);
  Eigen::Vector3d pos_world(0, 1, 2);
  Eigen::Vector3d pos_self = p3d.getPosInSelf(pos_world);

  EXPECT_NEAR(pos_self(0), 1, epsilon);
  EXPECT_NEAR(pos_self(1), 0, epsilon);
  EXPECT_NEAR(pos_self(2), 2, epsilon);
}

TEST(get3DPose, rotatedPose)
{
  Pose2DModel p2d;

  Eigen::VectorXd parameters = Eigen::VectorXd::Zero(3);
  parameters(2) = 90;
  p2d.setParameters(parameters);

  PoseModel p3d = p2d.get3DPose();

  Eigen::Vector3d pos_world(0, 1, 2);
  Eigen::Vector3d pos_self = p3d.getPosInSelf(pos_world);

  EXPECT_NEAR(pos_self(0), 1, epsilon);
  EXPECT_NEAR(pos_self(1), 0, epsilon);
  EXPECT_NEAR(pos_self(2), 2, epsilon);
}

TEST(get3DPose, rotAndTranslatePose)
{
  Pose2DModel p2d;

  Eigen::VectorXd parameters = Eigen::VectorXd::Zero(3);
  parameters(0) = -1;
  parameters(1) = 1;
  parameters(2) = 90;
  p2d.setParameters(parameters);

  PoseModel p3d = p2d.get3DPose();

  Eigen::Vector3d pos_world(0, 1, 2);
  Eigen::Vector3d pos_self = p3d.getPosInSelf(pos_world);

  EXPECT_NEAR(pos_self(0), 0, epsilon);
  EXPECT_NEAR(pos_self(1), -1, epsilon);
  EXPECT_NEAR(pos_self(2), 2, epsilon);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

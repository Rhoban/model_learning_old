#include "rhoban_model_learning/humanoid_models/pose_model.h"

#include <gtest/gtest.h>

using namespace std;
using namespace rhoban_model_learning;

static double epsilon = std::pow(10, -10);

TEST(defaultConstructor, defaultPose)
{
  PoseModel p;
  Eigen::VectorXd parameters = p.getParameters();
  EXPECT_EQ(parameters.rows(), 7);
  EXPECT_FLOAT_EQ(parameters(0), 0);
  EXPECT_FLOAT_EQ(parameters(1), 0);
  EXPECT_FLOAT_EQ(parameters(2), 0);
  EXPECT_FLOAT_EQ(parameters(3), 1);
  EXPECT_FLOAT_EQ(parameters(4), 0);
  EXPECT_FLOAT_EQ(parameters(5), 0);
  EXPECT_FLOAT_EQ(parameters(6), 0);
}

TEST(getPosInSelf, defaultPose)
{
  PoseModel p;
  Eigen::Vector3d pos_world(0, 1, 2);
  Eigen::Vector3d pos_self = p.getPosInSelf(pos_world);
  for (int d = 0; d < 3; d++)
  {
    EXPECT_FLOAT_EQ(pos_self(d), pos_world(d));
  }
}

TEST(getPosInSelf, offsetPose)
{
  PoseModel p;
  Eigen::VectorXd parameters = Eigen::VectorXd::Zero(7);
  parameters(2) = 1;  // Pose is at (0,0,1)
  p.setParameters(parameters);
  Eigen::Vector3d pos_world(0, 1, 2);
  Eigen::Vector3d pos_self = p.getPosInSelf(pos_world);
  EXPECT_NEAR(pos_self(0), 0, epsilon);
  EXPECT_NEAR(pos_self(1), 1, epsilon);
  EXPECT_NEAR(pos_self(2), 1, epsilon);
}

TEST(getPosInSelf, rotatedPose)
{
  PoseModel p;
  Eigen::VectorXd parameters = Eigen::VectorXd::Zero(7);
  // Rotation of PI/2 around x-axis
  Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()));
  parameters(3) = q.w();
  parameters(4) = q.x();
  parameters(5) = q.y();
  parameters(6) = q.z();
  p.setParameters(parameters);
  Eigen::Vector3d pos_world(0, 1, 2);
  Eigen::Vector3d pos_self = p.getPosInSelf(pos_world);
  EXPECT_NEAR(pos_self(0), 0, epsilon);
  EXPECT_NEAR(pos_self(1), 2, epsilon);
  EXPECT_NEAR(pos_self(2), -1, epsilon);
}

TEST(getPosInSelf, rotAndTranslatePose)
{
  PoseModel p;
  Eigen::VectorXd parameters = Eigen::VectorXd::Zero(7);
  // Rotation of PI/2 around z-axis + pos = (0,2,0)
  parameters(1) = 2;
  Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));
  parameters(3) = q.w();
  parameters(4) = q.x();
  parameters(5) = q.y();
  parameters(6) = q.z();
  p.setParameters(parameters);
  Eigen::Vector3d pos_world(0, 0, 0);
  Eigen::Vector3d pos_self = p.getPosInSelf(pos_world);
  EXPECT_NEAR(-2, pos_self(0), epsilon);
  EXPECT_NEAR(0, pos_self(1), epsilon);
  EXPECT_NEAR(0, pos_self(2), epsilon);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

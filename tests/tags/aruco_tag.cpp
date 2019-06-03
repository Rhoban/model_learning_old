#include "rhoban_model_learning/tags/aruco_tag.h"

#include <gtest/gtest.h>

using namespace std;
using namespace rhoban_model_learning;

static double epsilon = std::pow(10, -10);

TEST(constructor, constructor)
{
  ArucoTag tag(1, 0.2, Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
  EXPECT_EQ(tag.marker_id, 1);
  EXPECT_FLOAT_EQ(tag.marker_size, 0.2);
  for (int d = 0; d < 3; d++)
  {
    EXPECT_FLOAT_EQ(tag.marker_center(d), 0);
  }
  EXPECT_EQ(tag.orientation.w(), 1);
  EXPECT_EQ(tag.orientation.x(), 0);
  EXPECT_EQ(tag.orientation.y(), 0);
  EXPECT_EQ(tag.orientation.z(), 0);
}

TEST(getCorner, no_tranformation)
{
  ArucoTag tag(1, 0.2, Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
  Eigen::Vector3d corner;
  double half_size = 0.1;

  corner = tag.getCorner(0);
  EXPECT_EQ(corner(0), -half_size);
  EXPECT_EQ(corner(1), -half_size);
  EXPECT_EQ(corner(2), 0);

  corner = tag.getCorner(1);
  EXPECT_EQ(corner(0), half_size);
  EXPECT_EQ(corner(1), -half_size);
  EXPECT_EQ(corner(2), 0);

  corner = tag.getCorner(2);
  EXPECT_EQ(corner(0), -half_size);
  EXPECT_EQ(corner(1), half_size);
  EXPECT_EQ(corner(2), 0);

  corner = tag.getCorner(3);
  EXPECT_EQ(corner(0), half_size);
  EXPECT_EQ(corner(1), half_size);
  EXPECT_EQ(corner(2), 0);
}

// TEST(get3DPose, defaultPose)
// {
//   Pose2DModel p2d;
//   PoseModel p3d = p2d.get3DPose();
//   Eigen::Vector3d pos_world(0, 1, 2);
//   Eigen::Vector3d pos_self = p3d.getPosInSelf(pos_world);
//   for (int d = 0; d < 3; d++)
//   {
//     EXPECT_FLOAT_EQ(pos_self(d), pos_world(d));
//   }
// }

// TEST(get3DPose, offsetPose)
// {
//   Pose2DModel p2d;
//   PoseModel p3d = p2d.get3DPose();

//   Eigen::VectorXd parameters = Eigen::VectorXd::Zero(7);
//   parameters(0) = -1;
//   parameters(1) = 1;  // Pose is at (-1,1)
//   p3d.setParameters(parameters);
//   Eigen::Vector3d pos_world(0, 1, 2);
//   Eigen::Vector3d pos_self = p3d.getPosInSelf(pos_world);

//   EXPECT_NEAR(pos_self(0), 1, epsilon);
//   EXPECT_NEAR(pos_self(1), 0, epsilon);
//   EXPECT_NEAR(pos_self(2), 2, epsilon);
// }

// TEST(get3DPose, rotatedPose)
// {
//   Pose2DModel p2d;
//   PoseModel p3d = p2d.get3DPose();

//   Eigen::VectorXd parameters = Eigen::VectorXd::Zero(7);
//   parameters(2) = 90;
//   p3d.setParameters(parameters);

//   Eigen::Vector3d pos_world(0, 1, 2);
//   Eigen::Vector3d pos_self = p3d.getPosInSelf(pos_world);

//   EXPECT_NEAR(pos_self(0), 1, epsilon);
//   EXPECT_NEAR(pos_self(1), 1, epsilon);
//   EXPECT_NEAR(pos_self(2), 2, epsilon);
// }

// TEST(get3DPose, rotAndTranslatePose)
// {
//   Pose2DModel p2d;
//   PoseModel p3d = p2d.get3DPose();

//   Eigen::VectorXd parameters = Eigen::VectorXd::Zero(7);
//   parameters(0) = -1;
//   parameters(1) = 1;
//   parameters(2) = 90;
//   p3d.setParameters(parameters);

//   Eigen::Vector3d pos_world(0, 1, 2);
//   Eigen::Vector3d pos_self = p3d.getPosInSelf(pos_world);

//   EXPECT_NEAR(pos_self(0), 0, epsilon);
//   EXPECT_NEAR(pos_self(1), -1, epsilon);
//   EXPECT_NEAR(pos_self(2), 2, epsilon);
// }

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

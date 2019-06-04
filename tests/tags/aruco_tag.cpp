#include "rhoban_model_learning/tags/aruco_tag.h"

#include <gtest/gtest.h>

using namespace std;
using namespace rhoban_model_learning;

// static double epsilon = std::pow(10, -10);

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

TEST(getCorner, noTranformation)
{
  ArucoTag tag(1, 0.2, Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
  Eigen::Vector3d corner;
  double half_size = 0.1;

  corner = tag.getCorner(0);
  EXPECT_FLOAT_EQ(corner(0), 0);
  EXPECT_FLOAT_EQ(corner(1), -half_size);
  EXPECT_FLOAT_EQ(corner(2), half_size);

  corner = tag.getCorner(1);
  EXPECT_FLOAT_EQ(corner(0), 0);
  EXPECT_FLOAT_EQ(corner(1), half_size);
  EXPECT_FLOAT_EQ(corner(2), half_size);

  corner = tag.getCorner(2);
  EXPECT_FLOAT_EQ(corner(0), 0);
  EXPECT_FLOAT_EQ(corner(1), half_size);
  EXPECT_FLOAT_EQ(corner(2), -half_size);

  corner = tag.getCorner(3);
  EXPECT_FLOAT_EQ(corner(0), 0);
  EXPECT_FLOAT_EQ(corner(1), -half_size);
  EXPECT_FLOAT_EQ(corner(2), -half_size);
}

TEST(getCorner, translation)
{
  ArucoTag tag(1, 0.2, Eigen::Vector3d(1, 2, 3), Eigen::Quaterniond::Identity());
  Eigen::Vector3d corner;
  double half_size = 0.1;

  corner = tag.getCorner(0);
  EXPECT_FLOAT_EQ(corner(0), 1);
  EXPECT_FLOAT_EQ(corner(1), 2 - half_size);
  EXPECT_FLOAT_EQ(corner(2), 3 + half_size);

  corner = tag.getCorner(1);
  EXPECT_FLOAT_EQ(corner(0), 1);
  EXPECT_FLOAT_EQ(corner(1), 2 + half_size);
  EXPECT_FLOAT_EQ(corner(2), 3 + half_size);

  corner = tag.getCorner(2);
  EXPECT_FLOAT_EQ(corner(0), 1);
  EXPECT_FLOAT_EQ(corner(1), 2 + half_size);
  EXPECT_FLOAT_EQ(corner(2), 3 - half_size);

  corner = tag.getCorner(3);
  EXPECT_FLOAT_EQ(corner(0), 1);
  EXPECT_FLOAT_EQ(corner(1), 2 - half_size);
  EXPECT_FLOAT_EQ(corner(2), 3 - half_size);
}

TEST(getCorner, rotation)
{
  ArucoTag tag(1, 0.2, Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond(1, 1, 0, 0));
  Eigen::Vector3d corner;
  double half_size = 0.1;

  corner = tag.getCorner(0);
  EXPECT_FLOAT_EQ(corner(0), 0);
  EXPECT_FLOAT_EQ(corner(1), -half_size);
  EXPECT_FLOAT_EQ(corner(2), -half_size);

  corner = tag.getCorner(1);
  EXPECT_FLOAT_EQ(corner(0), 0);
  EXPECT_FLOAT_EQ(corner(1), -half_size);
  EXPECT_FLOAT_EQ(corner(2), half_size);

  corner = tag.getCorner(2);
  EXPECT_FLOAT_EQ(corner(0), 0);
  EXPECT_FLOAT_EQ(corner(1), half_size);
  EXPECT_FLOAT_EQ(corner(2), half_size);

  corner = tag.getCorner(3);
  EXPECT_FLOAT_EQ(corner(0), 0);
  EXPECT_FLOAT_EQ(corner(1), half_size);
  EXPECT_FLOAT_EQ(corner(2), -half_size);
}

TEST(getCorner, translationAndRotation)
{
  ArucoTag tag(1, 0.2, Eigen::Vector3d(1, 2, 3), Eigen::Quaterniond(1, 0, -1, 0));
  Eigen::Vector3d corner;
  double half_size = 0.1;

  corner = tag.getCorner(0);
  EXPECT_FLOAT_EQ(corner(0), 1 - half_size);
  EXPECT_FLOAT_EQ(corner(1), 2 - half_size);
  EXPECT_FLOAT_EQ(corner(2), 3);

  corner = tag.getCorner(1);
  EXPECT_FLOAT_EQ(corner(0), 1 - half_size);
  EXPECT_FLOAT_EQ(corner(1), 2 + half_size);
  EXPECT_FLOAT_EQ(corner(2), 3);

  corner = tag.getCorner(2);
  EXPECT_FLOAT_EQ(corner(0), 1 + half_size);
  EXPECT_FLOAT_EQ(corner(1), 2 + half_size);
  EXPECT_FLOAT_EQ(corner(2), 3);

  corner = tag.getCorner(3);
  EXPECT_FLOAT_EQ(corner(0), 1 + half_size);
  EXPECT_FLOAT_EQ(corner(1), 2 - half_size);
  EXPECT_FLOAT_EQ(corner(2), 3);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

#include "rhoban_model_learning/humanoid_models/pose_model.h"

#include <rhoban_utils/angle.h>
#include <gtest/gtest.h>

using namespace std;
using namespace rhoban_model_learning;

static double epsilon = std::pow(10, -10);
string getAbsoluteTestFilePrefix()
{
  string filePath = __FILE__;
  string currentDirPath = filePath.substr(0, filePath.rfind("/"));
  return currentDirPath + "/../ressources/tags/";
}

TEST(defaultConstructor, defaultPose)
{
  PoseModel p;
  p.setMode(PoseModel::Mode::Quaternion);
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
  p.setMode(PoseModel::Mode::Quaternion);
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
  p.setMode(PoseModel::Mode::Quaternion);
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
  p.setMode(PoseModel::Mode::Quaternion);
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
  p.setMode(PoseModel::Mode::Quaternion);
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

TEST(rpy_mode, initialisation)
{
  PoseModel p;
  Eigen::VectorXd parameters = p.getParameters();
  EXPECT_EQ(parameters.rows(), 6);
  EXPECT_FLOAT_EQ(parameters(0), 0);
  EXPECT_FLOAT_EQ(parameters(1), 0);
  EXPECT_FLOAT_EQ(parameters(2), 0);
  EXPECT_FLOAT_EQ(parameters(3), 0);
  EXPECT_FLOAT_EQ(parameters(4), 0);
  EXPECT_FLOAT_EQ(parameters(5), 0);
}

TEST(rpy_mode, compatibility_quaternion_mode)
{
  PoseModel p_rpy;
  PoseModel p_quat;
  p_quat.setMode(PoseModel::Mode::Quaternion);

  // Radians
  double roll = rhoban_utils::deg2rad(45);
  double pitch = rhoban_utils::deg2rad(-30);
  double yaw = rhoban_utils::deg2rad(70);

  Eigen::VectorXd p_rpy_params(6);
  p_rpy_params(0) = 1;
  p_rpy_params(1) = 2;
  p_rpy_params(2) = 3;
  p_rpy_params(3) = roll;
  p_rpy_params(4) = pitch;
  p_rpy_params(5) = yaw;

  Eigen::Quaterniond q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
                         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
  Eigen::VectorXd p_quat_params(7);
  p_rpy_params(0) = 1;
  p_rpy_params(1) = 2;
  p_rpy_params(2) = 3;
  p_rpy_params(3) = q.w();
  p_rpy_params(4) = q.x();
  p_rpy_params(5) = q.y();
  p_rpy_params(6) = q.z();

  Eigen::Matrix3d rot_rpy = p_rpy.getRotationFromSelf();
  Eigen::Matrix3d rot_quat = p_quat.getRotationFromSelf();
  EXPECT_FLOAT_EQ(rot_rpy(0, 0), rot_quat(0, 0));
  EXPECT_FLOAT_EQ(rot_rpy(0, 1), rot_quat(0, 1));
  EXPECT_FLOAT_EQ(rot_rpy(0, 2), rot_quat(0, 2));
  EXPECT_FLOAT_EQ(rot_rpy(1, 0), rot_quat(1, 0));
  EXPECT_FLOAT_EQ(rot_rpy(1, 1), rot_quat(1, 1));
  EXPECT_FLOAT_EQ(rot_rpy(1, 2), rot_quat(1, 2));
  EXPECT_FLOAT_EQ(rot_rpy(2, 0), rot_quat(2, 0));
  EXPECT_FLOAT_EQ(rot_rpy(2, 1), rot_quat(2, 1));
  EXPECT_FLOAT_EQ(rot_rpy(2, 2), rot_quat(2, 2));
}

TEST(quat_mode, loadFile)
{
  PoseModel p;
  p.loadFile(getAbsoluteTestFilePrefix() + "pose_model_0.json");
  EXPECT_TRUE(p.mode == PoseModel::Mode::Quaternion);
  EXPECT_FLOAT_EQ(p.pos(0), 1);
  EXPECT_FLOAT_EQ(p.pos(1), 2);
  EXPECT_FLOAT_EQ(p.pos(2), 3);
  EXPECT_FLOAT_EQ(p.orientation(0), 1);
  EXPECT_FLOAT_EQ(p.orientation(1), 0);
  EXPECT_FLOAT_EQ(p.orientation(2), 1);
  EXPECT_FLOAT_EQ(p.orientation(3), 0);
}

TEST(rpy_mode, loadFile)
{
  PoseModel p;
  p.loadFile(getAbsoluteTestFilePrefix() + "pose_model_1.json");
  EXPECT_TRUE(p.mode == PoseModel::Mode::RPY);
  EXPECT_FLOAT_EQ(p.pos(0), 1);
  EXPECT_FLOAT_EQ(p.pos(1), 2);
  EXPECT_FLOAT_EQ(p.pos(2), 3);
  EXPECT_FLOAT_EQ(p.orientation(0), 30);
  EXPECT_FLOAT_EQ(p.orientation(1), 90);
  EXPECT_FLOAT_EQ(p.orientation(2), 20);
}

TEST(getQuaternion, getQuaternion)
{
  PoseModel p;
  p.setMode(PoseModel::Mode::Quaternion);
  Eigen::Quaterniond quat_in(1, 0, 1, 0);
  p.setOrientation(quat_in);
  Eigen::Quaterniond quat_out = p.getQuaternion();

  EXPECT_FLOAT_EQ(quat_out.w(), 1 / sqrt(2));
  EXPECT_FLOAT_EQ(quat_out.x(), 0);
  EXPECT_FLOAT_EQ(quat_out.y(), 1 / sqrt(2));
  EXPECT_FLOAT_EQ(quat_out.z(), 0);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

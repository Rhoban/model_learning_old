#include "rhoban_model_learning/humanoid_models/pose_model.h"

#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
#include <rhoban_utils/util.h>
#include <rhoban_utils/angle.h>

#include <iostream>
namespace rhoban_model_learning
{
PoseModel::PoseModel()
  : Model(), pos(Eigen::Vector3d::Zero()), orientation(Eigen::VectorXd::Zero(3)), mode(PoseModel::Mode::RPY)
{
}

PoseModel::PoseModel(const PoseModel& other)
  : Model(other), pos(other.pos), orientation(other.orientation), mode(other.mode)
{
}

Eigen::Vector3d PoseModel::getPosInSelf(const Eigen::Vector3d& pos_in_world) const
{
  Eigen::Vector3d vec_in_world = pos_in_world - pos;
  return getRotationToSelf() * vec_in_world;
}

Eigen::Vector3d PoseModel::getPosFromSelf(const Eigen::Vector3d& pos_in_self) const
{
  return pos + getRotationFromSelf() * pos_in_self;
}

Eigen::Matrix<double, 3, 3> PoseModel::getRotationFromSelf() const
{
  Eigen::Matrix3d rotation;
  if (mode == PoseModel::Mode::RPY)
  {
    double roll = rhoban_utils::deg2rad(orientation(0));
    double pitch = rhoban_utils::deg2rad(orientation(1));
    double yaw = rhoban_utils::deg2rad(orientation(2));
    rotation = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
               Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
  }
  else if (mode == PoseModel::Mode::Quaternion)
  {
    rotation = Eigen::Quaterniond(orientation(0), orientation(1), orientation(2), orientation(3))
                   .normalized()
                   .toRotationMatrix();
  }
  return rotation;
}

Eigen::Quaterniond PoseModel::getQuaternion() const
{
  Eigen::Quaterniond quat(getRotationFromSelf());
  return quat;
}

Eigen::Affine3d PoseModel::getAffine3d() const
{
  Eigen::Matrix3d rotation = getRotationFromSelf();

  Eigen::Matrix4d mat;  // Your Transformation Matrix
  mat.setIdentity();    // Set to Identity to make bottom row of Matrix 0,0,0,1
  mat.block<3, 3>(0, 0) = rotation;
  mat.block<3, 1>(0, 3) = pos.transpose();

  Eigen::Affine3d trans(mat);
  return trans;
}

Eigen::Matrix<double, 3, 3> PoseModel::getRotationToSelf() const
{
  return getRotationFromSelf().transpose();
}

int PoseModel::getParametersSize() const
{
  int size = 0;
  if (mode == PoseModel::Mode::RPY)
  {
    size = 6;
  }
  else if (mode == PoseModel::Mode::Quaternion)
  {
    size = 7;
  }
  return size;
}

Eigen::VectorXd PoseModel::getParameters() const
{
  Eigen::VectorXd parameters;
  if (mode == PoseModel::Mode::RPY)
  {
    parameters.resize(6);
    parameters.segment(0, 3) = pos;
    parameters.segment(3, 3) = orientation;
  }
  else if (mode == PoseModel::Mode::Quaternion)
  {
    parameters.resize(7);
    parameters.segment(0, 3) = pos;
    parameters.segment(3, 4) = orientation;
  }
  return parameters;
}

void PoseModel::setParameters(const Eigen::VectorXd& new_params)
{
  setPosition(new_params.segment(0, 3));
  setOrientation(new_params.segment(3, new_params.rows() - 3));
}

void PoseModel::setPosition(const Eigen::Vector3d pos_)
{
  pos = pos_;
}

void PoseModel::setOrientation(const Eigen::Quaterniond quat)
{
  if (mode != PoseModel::Mode::Quaternion)
  {
    throw std::runtime_error(DEBUG_INFO + " only Quaternion modes is implemented for this method.");
  }
  orientation(0) = quat.w();
  orientation(1) = quat.x();
  orientation(2) = quat.y();
  orientation(3) = quat.z();
}

void PoseModel::setOrientation(const Eigen::VectorXd orientation_)
{
  if (orientation.rows() != orientation_.rows())
  {
    throw std::runtime_error(DEBUG_INFO + " invalid size for new_params, expecting " +
                             std::to_string(getParametersSize()) + ", got " + std::to_string(orientation.rows()));
  }
  orientation = orientation_;
}

void PoseModel::setFromOpenCV(const cv::Mat r_vec, cv::Mat t_vec)
{
  if (mode != PoseModel::Mode::Quaternion)
  {
    throw std::runtime_error(DEBUG_INFO + " only Quaternion modes is implemented for this method.");
  }
  Eigen::Vector3d position;
  cv::cv2eigen(t_vec, position);
  setPosition(position);

  cv::Mat r_mat_cv;
  cv::Rodrigues(r_vec, r_mat_cv);
  Eigen::Matrix3d r_mat_eigen;
  cv::cv2eigen(r_mat_cv, r_mat_eigen);
  setOrientation(Eigen::Quaterniond(r_mat_eigen));
}

std::vector<std::string> PoseModel::getParametersNames() const
{
  std::vector<std::string> parameters_name;
  if (mode == PoseModel::Mode::RPY)
  {
    parameters_name = { "x", "y", "z", "roll", "pitch", "yaw" };
  }
  else if (mode == PoseModel::Mode::Quaternion)
  {
    parameters_name = { "x", "y", "z", "qw", "qx", "qy", "qz" };
  }
  return parameters_name;
}

void PoseModel::setMode(const PoseModel::Mode mode_)
{
  if (mode == mode_)
  {
    return;
  }
  mode = mode_;

  if (mode == PoseModel::Mode::RPY)
  {
    orientation.resize(3);
    orientation(0) = 0;
    orientation(1) = 0;
    orientation(2) = 0;
  }
  else if (mode == PoseModel::Mode::Quaternion)
  {
    orientation.resize(4);
    orientation(0) = 1;
    orientation(1) = 0;
    orientation(2) = 0;
    orientation(3) = 0;
  }
}

Json::Value PoseModel::toJson() const
{
  Json::Value v;
  v["pos"] = rhoban_utils::vector2Json(pos);
  if (mode == PoseModel::Mode::RPY)
  {
    // Angles are stored in deg
    Eigen::Vector3d angles_normalized;
    angles_normalized(0) = rhoban_utils::normalizeDeg(orientation(0));
    angles_normalized(1) = rhoban_utils::normalizeDeg(orientation(1));
    angles_normalized(2) = rhoban_utils::normalizeDeg(orientation(2));
    v["rpy"] = rhoban_utils::vector2Json(angles_normalized);
  }
  else if (mode == PoseModel::Mode::Quaternion)
  {
    Eigen::Vector4d orientation_ = orientation;
    v["quaternion"] = rhoban_utils::vector2Json(orientation_);
  }
  return v;
}

void PoseModel::fromJson(const Json::Value& v, const std::string& dir_name)
{
  (void)dir_name;
  rhoban_utils::tryReadEigen(v, "pos", &pos);
  if (v.isObject() && v.isMember("rpy"))
  {
    setMode(PoseModel::Mode::RPY);

    Eigen::Vector3d angles = rhoban_utils::readEigen<3, 1>(v, "rpy");
    double roll = rhoban_utils::normalizeDeg(angles(0));
    double pitch = rhoban_utils::normalizeDeg(angles(1));
    double yaw = rhoban_utils::normalizeDeg(angles(2));
    Eigen::Vector3d angles_normalized(roll, pitch, yaw);
    setOrientation(angles_normalized);
  }
  else if (v.isObject() && v.isMember("quaternion"))
  {
    setMode(PoseModel::Mode::Quaternion);
    setOrientation(rhoban_utils::readEigen<4, 1>(v, "quaternion"));
  }
}

std::string PoseModel::getClassName() const
{
  return "PoseModel";
}

std::unique_ptr<Model> PoseModel::clone() const
{
  return std::unique_ptr<Model>(new PoseModel(*this));
}
}  // namespace rhoban_model_learning

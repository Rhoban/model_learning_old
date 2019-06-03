#include "rhoban_model_learning/basic_models/pose_rpy_model.h"

#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
#include <rhoban_utils/util.h>
#include <rhoban_utils/angle.h>

namespace rhoban_model_learning
{
PoseRPYModel::PoseRPYModel()
  : Model(), pos(Eigen::Vector3d::Zero()), orientation(Eigen::Quaterniond(Eigen::Vector4d(0, 0, 0, 1)))
{
}

PoseRPYModel::PoseRPYModel(const PoseRPYModel& other) : Model(other), pos(other.pos), orientation(other.orientation)
{
}

Eigen::Vector3d PoseRPYModel::getPosInSelf(const Eigen::Vector3d& pos_in_world) const
{
  Eigen::Vector3d vec_in_world = pos_in_world - pos;
  return getRotationToSelf() * vec_in_world;
}

Eigen::Vector3d PoseRPYModel::getPosFromSelf(const Eigen::Vector3d& pos_in_self) const
{
  return pos + getRotationFromSelf() * pos_in_self;
}

Eigen::Matrix<double, 3, 3> PoseRPYModel::getRotationFromSelf() const
{
  return orientation.toRotationMatrix();
}

Eigen::Matrix<double, 3, 3> PoseRPYModel::getRotationToSelf() const
{
  return orientation.toRotationMatrix().transpose();
}

int PoseRPYModel::getParametersSize() const
{
  return 6;
}

Eigen::VectorXd PoseRPYModel::getParameters() const
{
  Eigen::VectorXd parameters(6);
  parameters.segment(0, 3) = pos;
  parameters.segment(3, 3) = orientation.toRotationMatrix().eulerAngles(0, 1, 2);
  return parameters;
}

void PoseRPYModel::setParameters(const Eigen::VectorXd& new_params)
{
  if (new_params.rows() != getParametersSize())
  {
    throw std::runtime_error(DEBUG_INFO + " invalid size for new_params, expecting 6, got " +
                             std::to_string(new_params.rows()));
  }
  pos = new_params.segment(0, 3);
  // w,x,y,z
  double roll = new_params(3);
  double pitch = new_params(4);
  double yaw = new_params(5);
  orientation = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
  orientation.normalize();
}

void PoseRPYModel::setPosition(const Eigen::Vector3d pos_)
{
  pos = pos_;
}

void PoseRPYModel::setOrientation(const Eigen::Quaterniond orientation_)
{
  orientation = orientation_;
  orientation.normalize();
}

void PoseRPYModel::setFromOpenCV(const cv::Mat r_vec, cv::Mat t_vec)
{
  cv::Mat r_mat_cv;
  cv::Rodrigues(r_vec, r_mat_cv);

  Eigen::Matrix3d r_mat_eigen;
  cv::cv2eigen(r_mat_cv, r_mat_eigen);
  orientation = Eigen::Quaterniond(r_mat_eigen);
  cv::cv2eigen(t_vec, pos);
}

std::vector<std::string> PoseRPYModel::getParametersNames() const
{
  return { "x", "y", "z", "roll", "pitch", "yaw" };
}

Json::Value PoseRPYModel::toJson() const
{
  Json::Value v;
  v["pos"] = rhoban_utils::vector2Json(pos);
  Eigen::Vector3d angles = orientation.toRotationMatrix().eulerAngles(0, 1, 2);
  v["angles"] = rhoban_utils::vector2Json(angles);
  return v;
}

void PoseRPYModel::fromJson(const Json::Value& v, const std::string& dir_name)
{
  (void)dir_name;
  rhoban_utils::tryReadEigen(v, "pos", &pos);
  Eigen::Vector3d angles = Eigen::Vector3d::Zero();
  rhoban_utils::tryReadEigen(v, "angles", &angles);
  double roll = rhoban_utils::deg2rad(angles(0));
  double pitch = rhoban_utils::deg2rad(angles(1));
  double yaw = rhoban_utils::deg2rad(angles(2));
  orientation = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
  orientation.normalize();
}

std::string PoseRPYModel::getClassName() const
{
  return "PoseRPYModel";
}

std::unique_ptr<Model> PoseRPYModel::clone() const
{
  return std::unique_ptr<Model>(new PoseRPYModel(*this));
}

}  // namespace rhoban_model_learning

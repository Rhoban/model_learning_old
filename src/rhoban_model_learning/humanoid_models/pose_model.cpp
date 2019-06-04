#include "rhoban_model_learning/humanoid_models/pose_model.h"

#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
#include <rhoban_utils/util.h>
#include <rhoban_utils/angle.h>

#include <iostream>
namespace rhoban_model_learning
{
PoseModel::PoseModel()
  : Model(), pos(Eigen::Vector3d::Zero()), orientation(Eigen::Quaterniond(1, 0, 0, 0)), mode(PoseModel::Mode::RPY)
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
  return orientation.toRotationMatrix();
}

Eigen::Matrix<double, 3, 3> PoseModel::getRotationToSelf() const
{
  return orientation.toRotationMatrix().transpose();
}

int PoseModel::getParametersSize() const
{
  if (mode == PoseModel::Mode::RPY)
  {
    return 6;
  }
  if (mode == PoseModel::Mode::Quaternion)
  {
    return 7;
  }
  else
  {
    throw std::runtime_error(DEBUG_INFO + " only RPY and Quaternion modes are implemented.");
  }
}

Eigen::VectorXd PoseModel::getParameters() const
{
  if (mode == PoseModel::Mode::RPY)
  {
    Eigen::VectorXd parameters(6);
    parameters.segment(0, 3) = pos;
    parameters.segment(3, 3) = orientation.toRotationMatrix().eulerAngles(0, 1, 2);
    return parameters;
  }
  else if (mode == PoseModel::Mode::Quaternion)
  {
    Eigen::VectorXd parameters(7);
    parameters.segment(0, 3) = pos;
    parameters(3) = orientation.w();
    parameters(4) = orientation.x();
    parameters(5) = orientation.y();
    parameters(6) = orientation.z();
    return parameters;
  }
  else
  {
    throw std::runtime_error(DEBUG_INFO + " only RPY and Quaternion modes are implemented.");
  }
}

void PoseModel::setParameters(const Eigen::VectorXd& new_params)
{
  if (new_params.rows() != getParametersSize())
  {
    throw std::runtime_error(DEBUG_INFO + " invalid size for new_params, expecting " +
                             std::to_string(getParametersSize()) + ", got " + std::to_string(new_params.rows()));
  }
  pos = new_params.segment(0, 3);
  // w,x,y,z
  if (mode == PoseModel::Mode::RPY)
  {
    double roll = new_params(3);
    double pitch = new_params(4);
    double yaw = new_params(5);
    orientation = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
                  Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
  }
  else if (mode == PoseModel::Mode::Quaternion)
  {
    orientation = Eigen::Quaterniond(new_params(3), new_params(4), new_params(5), new_params(6));
  }
  orientation.normalize();
}

void PoseModel::setPosition(const Eigen::Vector3d pos_)
{
  pos = pos_;
}

void PoseModel::setOrientation(const Eigen::Quaterniond orientation_)
{
  orientation = orientation_;
  orientation.normalize();
}

void PoseModel::setFromOpenCV(const cv::Mat r_vec, cv::Mat t_vec)
{
  cv::Mat r_mat_cv;
  cv::Rodrigues(r_vec, r_mat_cv);

  Eigen::Matrix3d r_mat_eigen;
  cv::cv2eigen(r_mat_cv, r_mat_eigen);
  orientation = Eigen::Quaterniond(r_mat_eigen);
  cv::cv2eigen(t_vec, pos);
}

std::vector<std::string> PoseModel::getParametersNames() const
{
  if (mode == PoseModel::Mode::RPY)
  {
    return { "x", "y", "z", "roll", "pitch", "yaw" };
  }
  else if (mode == PoseModel::Mode::Quaternion)
  {
    return { "x", "y", "z", "qw", "qx", "qy", "qz" };
  }
  else
  {
    throw std::runtime_error(DEBUG_INFO + " only RPY and Quaternion modes are implemented.");
  }
}

Json::Value PoseModel::toJson() const
{
  Json::Value v;
  v["pos"] = rhoban_utils::vector2Json(pos);
  if (mode == PoseModel::Mode::RPY)
  {
    Eigen::Vector3d angles = orientation.toRotationMatrix().eulerAngles(0, 1, 2);
    v["rpy"] = rhoban_utils::vector2Json(angles);
  }
  else if (mode == PoseModel::Mode::Quaternion)
  {
    v["orientation"] = rhoban_utils::val2Json<Eigen::Quaterniond>(orientation);
  }
  else
  {
    throw std::runtime_error(DEBUG_INFO + " only RPY and Quaternion modes are implemented.");
  }
  return v;
}

void PoseModel::fromJson(const Json::Value& v, const std::string& dir_name)
{
  (void)dir_name;
  rhoban_utils::tryReadEigen(v, "pos", &pos);
  if (v.isObject() && v.isMember("orientation"))
  {
    orientation = rhoban_utils::read<Eigen::Quaterniond>(v, "orientation");
    orientation.normalize();

    mode = PoseModel::Mode::Quaternion;
  }
  else if (v.isObject() && v.isMember("angle") && v.isMember("axis"))
  {
    Eigen::Vector3d axis = rhoban_utils::readEigen<3, 1>(v, "axis");
    axis.normalize();
    double angle_deg = 0;
    rhoban_utils::tryRead(v, "angle", &angle_deg);
    orientation = Eigen::Quaterniond(Eigen::AngleAxisd(rhoban_utils::deg2rad(angle_deg), axis));
    orientation.normalize();
    mode = PoseModel::Mode::Quaternion;  // AngleAxis parameters are note implemented
  }
  else if (v.isObject() && v.isMember("rpy"))
  {
    Eigen::Vector3d angles = rhoban_utils::readEigen<3, 1>(v, "rpy");
    double roll = rhoban_utils::deg2rad(angles(0));
    double pitch = rhoban_utils::deg2rad(angles(1));
    double yaw = rhoban_utils::deg2rad(angles(2));
    orientation = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
                  Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    orientation.normalize();

    mode = PoseModel::Mode::RPY;  // AngleAxis parameters are note implemented
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

#include "rhoban_model_learning/basic_models/pose_2d_model.h"

#include <rhoban_utils/util.h>

namespace rhoban_model_learning
{
Pose2DModel::Pose2DModel() : Model(), pos(Eigen::Vector2d::Zero()), angle(0)
{
}

Pose2DModel::Pose2DModel(const Pose2DModel& other) : Model(other), pos(other.pos), angle(other.angle)
{
}

int Pose2DModel::getParametersSize() const
{
  return 3;
}

Eigen::VectorXd Pose2DModel::getParameters() const
{
  Eigen::VectorXd parameters;
  parameters(0) = pos(0);
  parameters(1) = pos(1);
  parameters(2) = angle.getSignedValue();
  return parameters;
}

PoseModel Pose2DModel::get3DPose() const
{
  PoseModel pose;
  pose.setPosition(Eigen::Vector3d(pos(0), pos(1), 0));
  Eigen::Quaterniond q(Eigen::AngleAxisd(angle.getSignedValueRad(), Eigen::Vector3d(0, 0, 1)));
  pose.setOrientation(q);
}

void Pose2DModel::setParameters(const Eigen::VectorXd& new_params)
{
  pos = new_params.segment(0, 2);
  angle = rhoban_utils::Angle(new_params(2));
}

void Pose2DModel::setPosition(const Eigen::Vector2d pos_)
{
  pos = pos_;
}

void Pose2DModel::setAngle(double angle_)
{
  angle = rhoban_utils::Angle(angle_);
}
std::vector<std::string> Pose2DModel::getParametersNames() const
{
  return { "x", "y", "theta" };
}

Json::Value Pose2DModel::toJson() const
{
  Json::Value v;
  v["pos"] = rhoban_utils::vector2Json(pos);
  v["theta"] = rhoban_utils::val2Json(angle.getSignedValue());
  return v;
}

void Pose2DModel::fromJson(const Json::Value& v, const std::string& dir_name)
{
  (void)dir_name;
  rhoban_utils::tryReadEigen(v, "pos", &pos);
  rhoban_utils::tryRead(v, "theta", &angle);
}

std::string Pose2DModel::getClassName() const
{
  return "Pose2DModel";
}

std::unique_ptr<Model> Pose2DModel::clone() const
{
  return std::unique_ptr<Model>(new Pose2DModel(*this));
}

}  // namespace rhoban_model_learning

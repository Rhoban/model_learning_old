#pragma once

#include "rhoban_model_learning/model.h"
#include "rhoban_model_learning/humanoid_models/pose_model.h"
#include <rhoban_utils/angle.h>

#include <Eigen/Geometry>

namespace rhoban_model_learning
{
/// This model stocks the position and the orientation of an object in a 3d
/// world
class Pose2DModel : public Model
{
public:
  Pose2DModel();
  Pose2DModel(const Pose2DModel& other);

  int getParametersSize() const override;

  Eigen::VectorXd getParameters() const override;
  void setParameters(const Eigen::VectorXd& new_params) override;
  std::vector<std::string> getParametersNames() const override;

  void setPosition(const Eigen::Vector2d pos_);
  void setAngle(const double);

  const PoseModel get3DPose() const;

  Json::Value toJson() const override;
  void fromJson(const Json::Value& json_value, const std::string& dir_name) override;
  std::string getClassName() const override;

  virtual std::unique_ptr<Model> clone() const override;

  /// Position of the object inside a 2d frame
  Eigen::Vector2d pos;

  /// Orientation of the object inside a 2d frame
  rhoban_utils::Angle angle;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace rhoban_model_learning

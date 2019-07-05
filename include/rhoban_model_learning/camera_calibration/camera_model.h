#pragma once

#include "rhoban_model_learning/model.h"

#include "robot_model/camera_model.h"

namespace rhoban_model_learning
{
/// Simple Wrapper for rhoban::CameraModel
class CameraModel : public Model
{
public:
  CameraModel();
  CameraModel(const rhoban::CameraModel& other);
  CameraModel(const CameraModel& other);

  int getParametersSize() const override;
  Eigen::VectorXd getParameters() const override;
  void setParameters(const Eigen::VectorXd& new_params) override;
  std::vector<std::string> getParametersNames() const override;

  void fromJson(const Json::Value& json_value, const std::string& dir_name) override;
  Json::Value toJson() const override;
  std::string getClassName() const override;

  std::unique_ptr<Model> clone() const override;

  rhoban::CameraModel model;
};

}  // namespace rhoban_model_learning

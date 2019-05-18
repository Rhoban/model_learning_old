#pragma once

#include "rhoban_model_learning/model.h"

namespace rhoban_model_learning
{
/// A rotation model used to simulate joints and more particularly error in
/// joints
class RotationModel : public Model
{
public:
  RotationModel();
  RotationModel(const RotationModel& other);

  Eigen::Vector3d getRPY() const;

  int getParametersSize() const override;
  Eigen::VectorXd getParameters() const override;
  void setParameters(const Eigen::VectorXd& new_params) override;
  std::vector<std::string> getParametersNames() const override;

  void fromJson(const Json::Value& json_value, const std::string& dir_name) override;
  Json::Value toJson() const override;
  std::string getClassName() const override;

  std::unique_ptr<Model> clone() const override;

private:
  double roll;
  double pitch;
  double yaw;
};

}  // namespace rhoban_model_learning

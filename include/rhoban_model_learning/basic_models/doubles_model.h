#pragma once

#include "rhoban_model_learning/model.h"

namespace rhoban_model_learning
{
/// This model stocks the position and the orientation of an object in a 3d
/// world
class DoublesModel : public Model
{
public:
  DoublesModel();
  DoublesModel(const DoublesModel& other);

  int getParametersSize() const override;

  Eigen::VectorXd getParameters() const override;
  void setParameters(const Eigen::VectorXd& new_params) override;
  std::vector<std::string> getParametersNames() const override;

  Json::Value toJson() const override;
  void fromJson(const Json::Value& json_value, const std::string& dir_name) override;
  std::string getClassName() const override;

  virtual std::unique_ptr<Model> clone() const override;

  double getParameter(int i) const;

  /// List of parameters
  Eigen::VectorXd params;
};

}  // namespace rhoban_model_learning

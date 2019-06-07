#pragma once

#include "rhoban_model_learning/model_space.h"

#include <Eigen/Core>

namespace rhoban_model_learning
{
/// Default space is simply an hyperrectangle
class DefaultSpace : public ModelSpace
{
public:
  DefaultSpace();
  DefaultSpace(const DefaultSpace& other);

  Eigen::MatrixXd getParametersSpace(const Model& m, const ModelPrior& prior) const override;

  std::string getClassName() const override;
  void fromJson(const Json::Value& json_value, const std::string& dir_name) override;
  Json::Value toJson() const override;

  std::unique_ptr<ModelSpace> clone() const override;

private:
  Eigen::MatrixXd space;
};

}  // namespace rhoban_model_learning

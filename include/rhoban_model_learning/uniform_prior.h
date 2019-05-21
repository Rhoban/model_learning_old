#pragma once

#include "rhoban_model_learning/model_prior.h"

#include <Eigen/Core>

namespace rhoban_model_learning
{
/// Uniform prior is a vector of mins and a vector of maxs
class UniformPrior : public ModelPrior
{
public:
  UniformPrior();

  std::string getClassName() const override;
  void fromJson(const Json::Value& json_value, const std::string& dir_name) override;
  Json::Value toJson() const override;

private:
  Eigen::VectorXd mins;
  Eigen::VectorXd maxs;
};

}  // namespace rhoban_model_learning

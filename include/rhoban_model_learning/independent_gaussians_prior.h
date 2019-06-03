#pragma once

#include "rhoban_model_learning/model_prior.h"

#include <Eigen/Core>

namespace rhoban_model_learning
{
/// Default prior is a mean vector and a mean of stddevs
class IndependentGaussiansPrior : public ModelPrior
{
public:
  IndependentGaussiansPrior();

  Eigen::VectorXd getParametersInitialValues(const Model& m) const override;
  Eigen::VectorXd getParametersStdDev(const Model& m) const override;

  std::string getClassName() const override;
  void fromJson(const Json::Value& json_value, const std::string& dir_name) override;
  Json::Value toJson() const override;

private:
  Eigen::VectorXd means;
  Eigen::VectorXd deviations;
};

}  // namespace rhoban_model_learning

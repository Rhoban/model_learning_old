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
  IndependentGaussiansPrior(const IndependentGaussiansPrior& other);

  double getLogLikelihood(const Model& m, const std::set<int>& used_indices) const override;

  int getParametersSize() const override;
  Eigen::MatrixXd getParametersSpace() const override;

  void setInitialMean(const Model& m) override;

  std::string getClassName() const override;
  void fromJson(const Json::Value& json_value, const std::string& dir_name) override;
  Json::Value toJson() const override;

  std::unique_ptr<ModelPrior> clone() const override;

private:
  Eigen::VectorXd means;
  Eigen::VectorXd deviations;
};

}  // namespace rhoban_model_learning

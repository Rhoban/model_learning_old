#include "rhoban_model_learning/independent_gaussians_prior.h"

namespace rhoban_model_learning
{
IndependentGaussiansPrior::IndependentGaussiansPrior()
{
}

Eigen::VectorXd IndependentGaussiansPrior::getParametersInitialValues(const Model& m) const
{
  (void)m;
  return means;
}

Eigen::VectorXd IndependentGaussiansPrior::getParametersStdDev(const Model& m) const
{
  (void)m;
  return deviations;
}

std::string IndependentGaussiansPrior::getClassName() const
{
  return "IndependentGaussiansPrior";
}

void IndependentGaussiansPrior::fromJson(const Json::Value& json_value, const std::string& dir_name)
{
  (void)dir_name;
  means = rhoban_utils::readEigen<-1, 1>(json_value, "means");
  deviations = rhoban_utils::readEigen<-1, 1>(json_value, "deviations");
}

Json::Value IndependentGaussiansPrior::toJson() const
{
  Json::Value v;
  v["means"] = rhoban_utils::vector2Json(means);
  v["deviations"] = rhoban_utils::vector2Json(deviations);
  return v;
}

}  // namespace rhoban_model_learning

#include "rhoban_model_learning/deviation_based_space.h"

#include "rhoban_model_learning/model_prior.h"

namespace rhoban_model_learning
{
DeviationBasedSpace::DeviationBasedSpace() : ratio(3)
{
}

Eigen::MatrixXd DeviationBasedSpace::getParametersSpace(const Model& m, const ModelPrior& prior) const
{
  Eigen::VectorXd mean = prior.getParametersInitialValues(m);
  Eigen::VectorXd dev = prior.getParametersStdDev(m);
  Eigen::MatrixXd space(mean.rows(), 2);
  space.block(0, 0, mean.rows(), 1) = mean - dev * ratio;
  space.block(0, 1, mean.rows(), 1) = mean + dev * ratio;
  return space;
}

std::string DeviationBasedSpace::getClassName() const
{
  return "DeviationBasedSpace";
}

void DeviationBasedSpace::fromJson(const Json::Value& json_value, const std::string& dir_name)
{
  (void)dir_name;
  rhoban_utils::tryRead(json_value, "ratio", &ratio);
}

Json::Value DeviationBasedSpace::toJson() const
{
  Json::Value v;
  v["ratio"] = ratio;
  return v;
}

}  // namespace rhoban_model_learning

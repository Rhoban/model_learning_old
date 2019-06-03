#include "rhoban_model_learning/composite_prior.h"

#include "rhoban_model_learning/composite_model.h"
#include "rhoban_model_learning/model_prior_factory.h"

namespace rhoban_model_learning
{
CompositePrior::CompositePrior() : ModelPrior()
{
}

size_t CompositePrior::getNbParameters(const Model& m) const
{
  size_t total_rows = 0;
  for (const auto& entry : priors)
  {
    total_rows += entry.second->getParametersInitialValues(getSubModel(m, entry.first)).rows();
  }
  return total_rows;
}

const Model& CompositePrior::getSubModel(const Model& m, const std::string& name) const
{
  return (dynamic_cast<const CompositeModel&>(m)).getModel(name);
}

Eigen::VectorXd CompositePrior::getParametersInitialValues(const Model& m) const
{
  Eigen::VectorXd params(getNbParameters(m));
  int idx = 0;
  for (const auto& entry : priors)
  {
    Eigen::VectorXd local_params = entry.second->getParametersInitialValues(getSubModel(m, entry.first));
    params.segment(idx, local_params.rows()) = local_params;
    idx += local_params.rows();
  }
  return params;
}

Eigen::VectorXd CompositePrior::getParametersStdDev(const Model& m) const
{
  Eigen::VectorXd devs(getNbParameters(m));
  int idx = 0;
  for (const auto& entry : priors)
  {
    Eigen::VectorXd local_devs = entry.second->getParametersStdDev(getSubModel(m, entry.first));
    devs.segment(idx, local_devs.rows()) = local_devs;
    idx += local_devs.rows();
  }
  return devs;
}

Json::Value CompositePrior::toJson() const
{
  Json::Value v;
  for (const auto& entry : priors)
  {
    v["priors"][entry.first] = entry.second->toJson();
  }
  return v;
}

void CompositePrior::fromJson(const Json::Value& v, const std::string& dir_name)
{
  priors = ModelPriorFactory().readMap(v, "priors", dir_name);
}

std::string CompositePrior::getClassName() const
{
  return "CompositePrior";
}

}  // namespace rhoban_model_learning

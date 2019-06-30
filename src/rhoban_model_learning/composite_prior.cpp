#include "rhoban_model_learning/composite_prior.h"

#include "rhoban_model_learning/composite_model.h"
#include "rhoban_model_learning/model_prior_factory.h"

namespace rhoban_model_learning
{
CompositePrior::CompositePrior() : ModelPrior()
{
}

CompositePrior::CompositePrior(const CompositePrior& other) : ModelPrior()
{
  for (const auto& entry : other.priors)
  {
    priors[entry.first] = entry.second->clone();
  }
}

double CompositePrior::getLogLikelihood(const Model& m, const std::set<int>& used_indices) const
{
  const CompositeModel& model = dynamic_cast<const CompositeModel&>(m);
  double log_likelihood = 0.0;
  std::map<std::string, std::set<int>> indices_splitting = model.splitIndicesAmongSubModels(used_indices);
  for (const auto& entry : priors)
  {
    log_likelihood += entry.second->getLogLikelihood(getSubModel(m, entry.first), indices_splitting[entry.first]);
  }

  return log_likelihood;
}

void CompositePrior::setInitialMean(const Model& m)
{
  const CompositeModel& composite_model = dynamic_cast<const CompositeModel&>(m);
  for (const auto& entry : priors)
  {
    entry.second->setInitialMean(composite_model.getModel(entry.first));
  }
}

const Model& CompositePrior::getSubModel(const Model& m, const std::string& name) const
{
  return (dynamic_cast<const CompositeModel&>(m)).getModel(name);
}

int CompositePrior::getParametersSize() const
{
  int size = 0;
  for (const auto& entry : priors)
  {
    size += entry.second->getParametersSize();
  }
  return size;
}

Eigen::MatrixXd CompositePrior::getParametersSpace() const
{
  Eigen::MatrixXd total_space(getParametersSize(), 2);
  int offset = 0;
  for (const auto& entry : priors)
  {
    Eigen::MatrixXd space = entry.second->getParametersSpace();

    int size = entry.second->getParametersSize();
    total_space.block(offset, 0, size, 2) = space;

    offset += size;
  }

  return total_space;
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

std::unique_ptr<ModelPrior> CompositePrior::clone() const
{
  return std::unique_ptr<ModelPrior>(new CompositePrior(*this));
}

}  // namespace rhoban_model_learning

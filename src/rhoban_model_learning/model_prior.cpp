#include "rhoban_model_learning/model_prior.h"
#include "rhoban_model_learning/model_prior_factory.h"

#include "rhoban_model_learning/model.h"
#include "rhoban_model_learning/tools.h"

#include "rhoban_utils/util.h"

#include <iostream>

namespace rhoban_model_learning
{
double ModelPrior::getLogLikelihood(const Model& m) const
{
  std::set<int> indices;
  for (int i = 0; i < m.getParametersSize(); i++)
  {
    indices.insert(i);
  }
  return getLogLikelihood(m, indices);
}

void ModelPrior::setInitialMean(const Model& m)
{
  (void)m;
  return;
}

void ModelPrior::updateDeviations(const Model& m, const std::vector<Eigen::VectorXd> parameters_values,
                                  const std::set<int> used_indices)
{
  (void)m;
  if (used_indices.size() != parameters_values.size())
  {
    throw std::runtime_error(DEBUG_INFO + "used_indices (size=" + std::to_string(used_indices.size()) +
                             ") and parameters_values (size=" + std::to_string(parameters_values.size()) +
                             ") should have the same size.");
  }
  return;
}

Eigen::VectorXd ModelPrior::addNoiseToParameters(const Model& m, const Eigen::VectorXd parameters_values,
                                                 const std::set<int> used_indices, std::default_random_engine* engine)
{
  std::cout << "Default adding noise." << std::endl;
  (void)m;
  (void)engine;
  if (used_indices.size() != parameters_values.size())
  {
    throw std::runtime_error(DEBUG_INFO + "used_indices (size=" + std::to_string(used_indices.size()) +
                             ") and parameters_values (size=" + std::to_string(parameters_values.size()) +
                             ") should have the same size.");
  }
  return parameters_values;
}

std::unique_ptr<ModelPrior> ModelPrior::clone() const
{
  Json::Value v = toJson();
  std::unique_ptr<ModelPrior> other = ModelPriorFactory().build(getClassName());
  other->fromJson(v, "./");
  return other;
}

}  // namespace rhoban_model_learning

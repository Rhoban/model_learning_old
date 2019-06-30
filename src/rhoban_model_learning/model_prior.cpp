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

std::unique_ptr<ModelPrior> ModelPrior::clone() const
{
  Json::Value v = toJson();
  std::unique_ptr<ModelPrior> other = ModelPriorFactory().build(getClassName());
  other->fromJson(v, "./");
  return other;
}

}  // namespace rhoban_model_learning

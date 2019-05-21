#include "rhoban_model_learning/model_prior_factory.h"

#include "rhoban_model_learning/composite_prior.h"
#include "rhoban_model_learning/default_prior.h"
#include "rhoban_model_learning/uniform_prior.h"

namespace rhoban_model_learning
{
ModelPriorFactory::ModelPriorFactory()
{
  registerBuilder("CompositePrior", []() { return std::unique_ptr<ModelPrior>(new CompositePrior); });
  registerBuilder("DefaultPrior", []() { return std::unique_ptr<ModelPrior>(new DefaultPrior); });
  registerBuilder("UniformPrior", []() { return std::unique_ptr<ModelPrior>(new UniformPrior); });
}

}  // namespace rhoban_model_learning

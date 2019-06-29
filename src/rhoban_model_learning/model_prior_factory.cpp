#include "rhoban_model_learning/model_prior_factory.h"

#include "rhoban_model_learning/composite_prior.h"
#include "rhoban_model_learning/independent_gaussians_prior.h"
#include "rhoban_model_learning/uniform_prior.h"

namespace rhoban_model_learning
{
ModelPriorFactory::ModelPriorFactory()
{
  registerBuilder("CompositePrior", []() { return std::unique_ptr<ModelPrior>(new CompositePrior); });
  registerBuilder("IndependentGaussiansPrior",
                  []() { return std::unique_ptr<ModelPrior>(new IndependentGaussiansPrior); });
  registerBuilder("UniformPrior", []() { return std::unique_ptr<ModelPrior>(new UniformPrior); });
}

}  // namespace rhoban_model_learning

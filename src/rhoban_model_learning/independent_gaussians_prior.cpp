#include "rhoban_model_learning/independent_gaussians_prior.h"

#include "rhoban_random/gaussian_distribution.h"
#include <rhoban_utils/util.h>

#include <iostream>

namespace rhoban_model_learning
{
IndependentGaussiansPrior::IndependentGaussiansPrior()
{
}

IndependentGaussiansPrior::IndependentGaussiansPrior(const IndependentGaussiansPrior& other)
  : means(other.means), deviations(other.deviations)
{
}

std::string IndependentGaussiansPrior::getClassName() const
{
  return "IndependentGaussiansPrior";
}

double IndependentGaussiansPrior::getLogLikelihood(const Model& m, const std::set<int>& used_indices) const
{
  Eigen::VectorXd parameters = m.getParameters();
  double log_likelihood = 0.0;
  for (int i : used_indices)
  {
    double stddev = deviations(i);
    if (stddev <= 0.0)
    {
      throw std::logic_error(DEBUG_INFO + "Negative or null stddev found for parameter '" + m.getParametersNames()[i] +
                             "'");
    }
    rhoban_random::GaussianDistribution distrib(means(i), stddev * stddev);
    log_likelihood += distrib.getLogLikelihood(parameters(i));
  }
  return log_likelihood;
}

int IndependentGaussiansPrior::getParametersSize() const
{
  return deviations.rows();
}

Eigen::MatrixXd IndependentGaussiansPrior::getParametersSpace() const
{
  Eigen::MatrixXd space(means.rows(), 2);

  // XXX : add again the ratio for the deviation
  space.block(0, 0, means.rows(), 1) = means - 2 * deviations;
  space.block(0, 1, means.rows(), 1) = means + 2 * deviations;

  return space;
}

void IndependentGaussiansPrior::fromJson(const Json::Value& json_value, const std::string& dir_name)
{
  (void)dir_name;
  deviations = rhoban_utils::readEigen<-1, 1>(json_value, "deviations");
}

void IndependentGaussiansPrior::setInitialMean(const Model& m)
{
  means = m.getParameters();
}

Json::Value IndependentGaussiansPrior::toJson() const
{
  Json::Value v;
  v["deviations"] = rhoban_utils::vector2Json(deviations);
  return v;
}

std::unique_ptr<ModelPrior> IndependentGaussiansPrior::clone() const
{
  return std::unique_ptr<ModelPrior>(new IndependentGaussiansPrior(*this));
}

}  // namespace rhoban_model_learning

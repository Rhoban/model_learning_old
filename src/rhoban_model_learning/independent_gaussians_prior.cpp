#include "rhoban_model_learning/independent_gaussians_prior.h"

#include "rhoban_random/multivariate_gaussian.h"

#include "rhoban_random/gaussian_distribution.h"
#include <rhoban_utils/util.h>
#include <rhoban_utils/stats/stats.h>

#include <iostream>

namespace rhoban_model_learning
{
IndependentGaussiansPrior::IndependentGaussiansPrior() : ratio(1.5)
{
}

IndependentGaussiansPrior::IndependentGaussiansPrior(const IndependentGaussiansPrior& other)
  : means(other.means), deviations(other.deviations), ratio(other.ratio)
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

  space.block(0, 0, means.rows(), 1) = means - ratio * deviations;
  space.block(0, 1, means.rows(), 1) = means + ratio * deviations;

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

void IndependentGaussiansPrior::updateDeviations(const Model& m, const std::vector<Eigen::VectorXd> parameters_values,
                                                 const std::set<int> used_indices)
{
  (void)m;

  if (used_indices.size() != parameters_values.size())
  {
    throw std::runtime_error(DEBUG_INFO + "used_indices (size=" + std::to_string(used_indices.size()) +
                             ") and parameters_values (size=" + std::to_string(parameters_values.size()) +
                             ") should have the same size.");
  }

  int i = 0;
  for (auto it = used_indices.begin(); it != used_indices.end(); it++)
  {
    Eigen::VectorXd vals = parameters_values[i];
    std::vector<double> vals_std(vals.data(), vals.data() + vals.size());
    deviations[*it] = rhoban_utils::standardDeviation(vals_std);
    i++;
  }
}

Eigen::VectorXd IndependentGaussiansPrior::addNoiseToParameters(const Model& m, const Eigen::VectorXd parameters_values,
                                                                const std::set<int> used_indices,
                                                                std::default_random_engine* engine)
{
  Eigen::VectorXd parameters_noised_values(parameters_values.size());
  int j = 0;
  for (int index : used_indices)
  {
    std::normal_distribution<double> distrib(parameters_values(j), deviations(index));
    double val = distrib(*engine);
    parameters_noised_values(j) = val;
    j++;
  }

  return parameters_noised_values;
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

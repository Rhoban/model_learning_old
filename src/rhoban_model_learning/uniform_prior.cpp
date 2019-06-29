#include "rhoban_model_learning/uniform_prior.h"

#include "rhoban_random/uniform_distribution.h"

#include <rhoban_utils/util.h>

namespace rhoban_model_learning
{
UniformPrior::UniformPrior()
{
}

UniformPrior::UniformPrior(const UniformPrior& other) : mins(other.mins), maxs(other.maxs)
{
}

std::string UniformPrior::getClassName() const
{
  return "UniformPrior";
}

double UniformPrior::getLogLikelihood(const Model& m, const std::set<int>& used_indices) const
{
  Eigen::VectorXd parameters = m.getParameters();
  double log_likelihood = 0.0;
  for (int i : used_indices)
  {
    rhoban_random::UniformDistribution distrib(mins(i), maxs(i));
    log_likelihood += distrib.getLogLikelihood(parameters(i));
  }
  return log_likelihood;
}

int UniformPrior::getParametersSize() const
{
  return mins.rows();
}

Eigen::MatrixXd UniformPrior::getParametersSpace() const
{
  Eigen::MatrixXd space(mins.rows(), 2);
  space.block(0, 0, mins.rows(), 1) = mins;
  space.block(0, 1, mins.rows(), 1) = maxs;

  return space;
}

void UniformPrior::fromJson(const Json::Value& json_value, const std::string& dir_name)
{
  (void)dir_name;
  mins = rhoban_utils::readEigen<-1, 1>(json_value, "mins");
  maxs = rhoban_utils::readEigen<-1, 1>(json_value, "maxs");
}

Json::Value UniformPrior::toJson() const
{
  Json::Value v;
  v["mins"] = rhoban_utils::vector2Json(mins);
  v["maxs"] = rhoban_utils::vector2Json(maxs);

  if (mins.rows() != maxs.rows())
  {
    throw std::runtime_error(DEBUG_INFO + "mins and maxs must have the same size.");
  }

  return v;
}

}  // namespace rhoban_model_learning

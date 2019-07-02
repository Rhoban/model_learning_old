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
    double sub_log_likelihood =
        entry.second->getLogLikelihood(getSubModel(m, entry.first), indices_splitting[entry.first]);
    // std::cout << "Prior: " << entry.first << " log likelihood: " << sub_log_likelihood << std::endl;
    log_likelihood += sub_log_likelihood;
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

void CompositePrior::updateDeviations(const Model& m, const std::vector<Eigen::VectorXd> parameters_values,
                                      const std::set<int> used_indices)
{
  if (used_indices.size() != parameters_values.size())
  {
    throw std::runtime_error(DEBUG_INFO + "used_indices (size=" + std::to_string(used_indices.size()) +
                             ") and parameters_values (size=" + std::to_string(parameters_values.size()) +
                             ") should have the same size.");
  }

  const CompositeModel& composite_model = dynamic_cast<const CompositeModel&>(m);
  std::map<std::string, std::set<int>> indices_splitting = composite_model.splitIndicesAmongSubModels(used_indices);
  for (const auto& entry : priors)
  {
    std::set<int> sub_used_indices = indices_splitting[entry.first];
    std::vector<Eigen::VectorXd> sub_parameters_values;

    int i = 0;
    for (auto it = used_indices.begin(); it != used_indices.end(); it++)
    {
      if (sub_used_indices.count(*it - composite_model.getOffset(entry.first)) > 0)
      {
        sub_parameters_values.push_back(parameters_values[i]);
      }
      i++;
    }

    entry.second->updateDeviations(getSubModel(composite_model, entry.first), sub_parameters_values, sub_used_indices);
  }
}

Eigen::VectorXd CompositePrior::addNoiseToParameters(const Model& m, const Eigen::VectorXd parameters_values,
                                                     const std::set<int> used_indices,
                                                     std::default_random_engine* engine)
{
  if (used_indices.size() != parameters_values.size())
  {
    throw std::runtime_error(DEBUG_INFO + "used_indices (size=" + std::to_string(used_indices.size()) +
                             ") and parameters_values (size=" + std::to_string(parameters_values.size()) +
                             ") should have the same size.");
  }

  Eigen::VectorXd parameters_noised_values(parameters_values.size());

  const CompositeModel& composite_model = dynamic_cast<const CompositeModel&>(m);
  std::map<std::string, std::set<int>> indices_splitting = composite_model.splitIndicesAmongSubModels(used_indices);
  for (const auto& entry : priors)
  {
    std::set<int> sub_used_indices = indices_splitting[entry.first];
    Eigen::VectorXd sub_parameters_values(sub_used_indices.size());

    int i = 0;
    int j = 0;
    for (auto it = used_indices.begin(); it != used_indices.end(); it++)
    {
      if (sub_used_indices.count(*it - composite_model.getOffset(entry.first)) > 0)
      {
        sub_parameters_values(j) = parameters_values(i);
        j++;
      }
      i++;
    }

    Eigen::VectorXd sub_parameters_noised_values = entry.second->addNoiseToParameters(
        getSubModel(composite_model, entry.first), sub_parameters_values, sub_used_indices, engine);
    i = 0;
    j = 0;
    for (auto it = used_indices.begin(); it != used_indices.end(); it++)
    {
      if (sub_used_indices.count(*it - composite_model.getOffset(entry.first)) > 0)
      {
        parameters_noised_values(i) = sub_parameters_noised_values(j);
        j++;
      }
      i++;
    }
  }
  return parameters_noised_values;
}

Json::Value CompositePrior::toJson() const
{
  Json::Value v;
  for (const auto& entry : priors)
  {
    v["priors"][entry.first] = entry.second->toFactoryJson();
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

#pragma once

#include "rhoban_utils/serialization/json_serializable.h"

#include <memory>

namespace rhoban_model_learning
{
class Model;

/// Describe the distribution over models
/// Note: currently only distribution with independent parameters are accepted
class ModelPrior : public rhoban_utils::JsonSerializable
{
public:
  /// Return the average value for each parameter in prior
  virtual Eigen::VectorXd getParametersInitialValues(const Model& m) const = 0;
  virtual Eigen::VectorXd getParametersInitialValues(const Model& m, const std::set<int>& used_indices) const;

  /// Return the standard deviation of each parameter according to the prior
  virtual Eigen::VectorXd getParametersStdDev(const Model& m) const = 0;
  virtual Eigen::VectorXd getParametersStdDev(const Model& m, const std::set<int>& used_indices) const;

  /// Return the loglikelihood of Model parameters according to the prior
  virtual double getLogLikelihood(const Model& m) const;

  /// Return the loglikelihood of Model parameters according to the prior based
  /// on parameters of the provided indices
  virtual double getLogLikelihood(const Model& m, const std::set<int>& used_indices) const;

  /// Default method for cloning is to serialize the object to Json and deserialize
  /// it which might be too time consuming for some classes
  virtual std::unique_ptr<ModelPrior> clone() const;
};

}  // namespace rhoban_model_learning

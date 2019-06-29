#pragma once

#include "rhoban_utils/serialization/json_serializable.h"
#include <rhoban_model_learning/model.h>

#include <Eigen/Core>
#include <memory>

namespace rhoban_model_learning
{
/// Describe the distribution over models
/// Note: currently only distribution with independent parameters are accepted
class ModelPrior : public rhoban_utils::JsonSerializable
{
public:
  /// Return the loglikelihood of Model parameters according to the prior
  virtual double getLogLikelihood(const Model& m) const;

  /// Return the loglikelihood of Model parameters according to the prior based
  /// on parameters of the provided indices
  virtual double getLogLikelihood(const Model& m, const std::set<int>& used_indices) const = 0;

  /// Return prior size
  virtual int getParametersSize() const = 0;

  /// Set the mean of the prior with initial values
  virtual void setInitialMean(const Model& m);

  /// Return parameters space
  virtual Eigen::MatrixXd getParametersSpace() const = 0;

  /// Default method for cloning is to serialize the object to Json and deserialize
  /// it which might be too time consuming for some classes
  virtual std::unique_ptr<ModelPrior> clone() const;
};

}  // namespace rhoban_model_learning

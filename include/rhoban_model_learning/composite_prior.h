#pragma once

#include "rhoban_model_learning/model_prior.h"

#include <memory>
#include <Eigen/Core>

namespace rhoban_model_learning
{
/// Default prior is a mean vector and a mean of stddevs
class CompositePrior : public ModelPrior
{
public:
  CompositePrior();
  CompositePrior(const CompositePrior& other);

  /// throws:
  /// - std::bad_cast if 'm' is not a composite model
  /// - std::out_of_range if 'name' is not a member of 'm'
  const Model& getSubModel(const Model& m, const std::string& name) const;

  double getLogLikelihood(const Model& m, const std::set<int>& used_indices) const;

  int getParametersSize() const override;
  Eigen::MatrixXd getParametersSpace() const override;

  std::string getClassName() const override;
  void fromJson(const Json::Value& json_value, const std::string& dir_name) override;
  Json::Value toJson() const override;

  std::unique_ptr<ModelPrior> clone() const override;

private:
  std::map<std::string, std::unique_ptr<ModelPrior>> priors;
};

}  // namespace rhoban_model_learning

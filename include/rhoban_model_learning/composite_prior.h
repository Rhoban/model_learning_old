#pragma once

#include "rhoban_model_learning/model_prior.h"

#include <memory>

namespace rhoban_model_learning
{
/// Default prior is a mean vector and a mean of stddevs
class CompositePrior : public ModelPrior
{
public:
  CompositePrior();

  size_t getNbParameters(const Model& m) const;

  /// throws:
  /// - std::bad_cast if 'm' is not a composite model
  /// - std::out_of_range if 'name' is not a member of 'm'
  const Model& getSubModel(const Model& m, const std::string& name) const;

  Eigen::VectorXd getParametersInitialValues(const Model& m) const override;
  Eigen::VectorXd getParametersStdDev(const Model& m) const override;

  std::string getClassName() const override;
  void fromJson(const Json::Value& json_value, const std::string& dir_name) override;
  Json::Value toJson() const override;

private:
  std::map<std::string, std::unique_ptr<ModelPrior>> priors;
};

}  // namespace rhoban_model_learning

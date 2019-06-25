#pragma once

#include "rhoban_model_learning/predictor.h"

namespace rhoban_model_learning
{
class LogMachinePredictor : public Predictor
{
public:
  LogMachinePredictor();

  Eigen::VectorXd predictObservation(const Input& input, const Model& model,
                                     std::default_random_engine* engine) const override;

  double computeLogLikelihood(const Sample& sample, const Model& model,
                              std::default_random_engine* engine) const override;

  Eigen::VectorXi getObservationsCircularity() const override;

  std::string getClassName() const override;

  virtual std::unique_ptr<Predictor> clone() const override;
};

}  // namespace rhoban_model_learning

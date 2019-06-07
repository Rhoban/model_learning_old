#pragma once

#include "rhoban_model_learning/predictor.h"

namespace rhoban_model_learning
{
/// Inputs should be PosesOptimizationInput which contain:
/// - Image number
/// - Aruco Id
///
/// Observations are positions of the marker in the image (x,y)
class PosesOptimizationPredictor : public Predictor
{
public:
  PosesOptimizationPredictor();

  Eigen::VectorXd predictObservation(const Input& input, const Model& model,
                                     std::default_random_engine* engine) const override;

  double computeLogLikelihood(const Sample& sample, const Model& model,
                              std::default_random_engine* engine) const override;

  Eigen::VectorXi getObservationsCircularity() const override;

  std::string getClassName() const override;

  void exportPredictionsToCSV(const Model& raw_model, const SampleVector& sample_vector, const std::string& filename,
                              char separator = ',') const override;

  virtual std::unique_ptr<Predictor> clone() const override;
};

}  // namespace rhoban_model_learning

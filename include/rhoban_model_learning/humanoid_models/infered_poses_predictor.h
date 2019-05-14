#pragma once

#include "rhoban_model_learning/predictor.h"

namespace rhoban_model_learning
{
/// Inputs should be InferedPosesInput which contain:
/// - a vector of Eigen::Vector3d representing the markers used to infer the camera position.
/// - marker Id
///
/// Observations are positions of the marker in the image (x,y)
class InferedPosesPredictor : public Predictor
{
public:
  InferedPosesPredictor();

  Eigen::VectorXd predictObservation(const Input& input, const Model& model,
                                     std::default_random_engine* engine) const override;

  double computeLogLikelihood(const Sample& sample, const Model& model,
                              std::default_random_engine* engine) const override;

  Eigen::VectorXi getObservationsCircularity() const override;

  std::string getClassName() const override;
};

}  // namespace rhoban_model_learning

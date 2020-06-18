#pragma once

#include "rhoban_model_learning/predictor.h"
#include "rhoban_model_learning/sample.h"

namespace rhoban_model_learning
{
/// Inputs should be InferedPosesHumanoidInput which contain:
/// - a vector of Eigen::Vector3d representing the markers used to infer the sheet position.
/// - the id of the marker we want to predict
/// - a PoseModel containing the first pose of the camera used to compute the sheet position
/// - a PoseModel containing the second pose of the camera used to compute the prediction
///
/// Observations are positions of the marker in the image (x,y)
class InferedPosesHumanoidPredictor : public Predictor
{
public:
  InferedPosesHumanoidPredictor();

  Eigen::VectorXd predictObservation(const Input& input, const Model& model,
                                     std::default_random_engine* engine) const override;

  double computeLogLikelihood(const Sample& sample, const Model& model,
                              std::default_random_engine* engine) const override;

  std::string getClassName() const override;

  void exportPredictionsToCSV(const Model& raw_model, const SampleVector& sample_vector, const std::string& filename,
                              char separator = ',') const override;
};

}  // namespace rhoban_model_learning

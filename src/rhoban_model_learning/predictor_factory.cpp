#include "rhoban_model_learning/predictor_factory.h"

#include "rhoban_model_learning/basic_models/linear_predictor.h"
#include "rhoban_model_learning/humanoid_models/poses_optimization_predictor.h"
#include "rhoban_model_learning/camera_calibration/infered_poses_predictor.h"

namespace rhoban_model_learning
{
PredictorFactory::PredictorFactory()
{
  registerBuilder("LinearPredictor", []() { return std::unique_ptr<Predictor>(new LinearPredictor); });
  registerBuilder("PosesOptimizationPredictor",
                  []() { return std::unique_ptr<Predictor>(new PosesOptimizationPredictor); });
  registerBuilder("InferedPosesPredictor", []() { return std::unique_ptr<Predictor>(new InferedPosesPredictor); });
}
}  // namespace rhoban_model_learning

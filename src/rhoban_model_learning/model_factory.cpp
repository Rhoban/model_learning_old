#include "rhoban_model_learning/model_factory.h"

#include "rhoban_model_learning/basic_models/linear_model.h"

#include <rhoban_model_learning/camera_calibration/camera_model.h>
#include "rhoban_model_learning/humanoid_models/multi_poses_model.h"
#include "rhoban_model_learning/humanoid_models/poses_optimization_model.h"
#include "rhoban_model_learning/humanoid_models/rotation_model.h"
#include "rhoban_model_learning/humanoid_models/vision_noise_model.h"

#include "rhoban_model_learning/tags/aruco_collection.h"
#include "rhoban_model_learning/tags/aruco_cube.h"
#include "rhoban_model_learning/tags/tags_sheet.h"

// TODO: integrate back
//#include "rhoban_model_learning/humanoid_models/vision_correction_model.h"
//#include "rhoban_model_learning/ball_models/position_predictor_factory.h"
//#include "rhoban_model_learning/ball_models/speed_estimator_factory.h"
//#include "rhoban_model_learning/ball_models/trajectory_predictor.h"

namespace rhoban_model_learning
{
ModelFactory::ModelFactory()
{
  // Basic models
  registerBuilder("LinearModel", []() { return std::unique_ptr<Model>(new LinearModel); });
  // Humanoid models
  registerBuilder("CameraModel", []() { return std::unique_ptr<Model>(new CameraModel); });
  registerBuilder("MultiPosesModel", []() { return std::unique_ptr<Model>(new MultiPosesModel); });
  registerBuilder("PosesOptimizationModel", []() { return std::unique_ptr<Model>(new PosesOptimizationModel); });
  registerBuilder("RotationModel", []() { return std::unique_ptr<Model>(new RotationModel); });
  registerBuilder("VisionNoiseModel", []() { return std::unique_ptr<Model>(new VisionNoiseModel); });
  // Tags models
  registerBuilder("ArucoCollection", []() { return std::unique_ptr<Model>(new ArucoCollection); });
  registerBuilder("ArucoCube", []() { return std::unique_ptr<Model>(new ArucoCube); });
  registerBuilder("TagsSheet", []() { return std::unique_ptr<Model>(new TagsSheet); });

  //  registerBuilder("VisionCorrectionModel",
  //                  []() { return std::unique_ptr<Model>(new VisionCorrectionModel); });
  //  registerBuilder("TrajectoryPredictor",
  //                  []() { return std::unique_ptr<Model>(new TrajectoryPredictor); });
  //  importJsonBuilders(PositionPredictorFactory());
  //  importJsonBuilders(SpeedEstimatorFactory());
}

}  // namespace rhoban_model_learning

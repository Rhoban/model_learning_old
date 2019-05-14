#include "rhoban_model_learning/data_set_reader_factory.h"

#include "rhoban_model_learning/default_data_set_reader.h"
#include "rhoban_model_learning/humanoid_models/poses_optimization_data_set_reader.h"
#include "rhoban_model_learning/humanoid_models/infered_poses_data_set_reader.h"
// TODO: integrate back readers
//#include "rhoban_model_learning/ball_models/trajectory_predictor.h"
//#include "rhoban_model_learning/humanoid_models/vision_correction_model.h"

namespace rhoban_model_learning
{
typedef std::unique_ptr<DataSetReader> PTR;

DataSetReaderFactory::DataSetReaderFactory()
{
  registerBuilder("DefaultDataSetReader", []() { return PTR(new DefaultDataSetReader); });
  registerBuilder("PosesOptimizationDataSetReader", []() { return PTR(new PosesOptimizationDataSetReader); });
  registerBuilder("InferedPosesDataSetReader", []() { return PTR(new InferedPosesDataSetReader); });
  //  registerBuilder("TrajectoryDataSetReader",
  //                  []() { return PTR(new TrajectoryPredictor::Reader); });
  //  registerBuilder("VisionDataSetReader",
  //                  []() { return PTR(new VisionCorrectionModel::VisionDataSetReader); });
}

}  // namespace rhoban_model_learning

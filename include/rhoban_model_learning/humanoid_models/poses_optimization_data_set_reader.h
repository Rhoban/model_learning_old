#pragma once

#include "rhoban_model_learning/data_set_reader.h"
#include <hl_monitoring/replay_image_provider.h>

namespace rhoban_model_learning
{
class PosesOptimizationDataSetReader : public DataSetReader
{
public:
  PosesOptimizationDataSetReader();

  /// Throw an error if:
  /// - number of `nb_training_images` and `nb_validation_images` is provided
  /// - The sum of `nb_training_images` and `nb_validation_tags` is higher
  ///   than the number of images with at least 'min_tags'
  virtual DataSet extractSamples(const std::string& file_path, std::default_random_engine* engine) const override;

  virtual std::string getClassName() const override;
  Json::Value toJson() const override;
  void fromJson(const Json::Value& v, const std::string& dir_name) override;

private:
  /// For each image, the number of tags used for training
  int nb_training_tags;

  /// For each image, the number of tags used for validation
  int nb_validation_tags;

  /// Printing debug information
  bool verbose;

  /// Camera from self poses
  hl_monitoring::ReplayImageProvider camera_from_self_poses;

  /// Head base from camera poses
  hl_monitoring::ReplayImageProvider camera_from_head_base_poses;
};

}  // namespace rhoban_model_learning

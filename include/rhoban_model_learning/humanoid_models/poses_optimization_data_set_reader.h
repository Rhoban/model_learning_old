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
  /// - none of `nb_training_images` and `nb_validation_images` is provided
  /// - The sum of `nb_training_images` and `nb_validation_tags` is higher
  ///   than the number of images with at least 'min_tags'
  virtual DataSet extractSamples(const std::string& file_path, std::default_random_engine* engine) const override;

  virtual std::string getClassName() const override;
  Json::Value toJson() const override;
  void fromJson(const Json::Value& v, const std::string& dir_name) override;

private:
  /// Number of images used for the data set
  int nb_images;

  /// For each image, the number of tags used for training
  int training_tags_per_image;

  /// For each image, the number of tags used for validation
  int validation_tags_per_image;

  /// Printing debug information
  bool verbose;

  ///
  hl_monitoring::ReplayImageProvider replay_image_provider;
};

}  // namespace rhoban_model_learning

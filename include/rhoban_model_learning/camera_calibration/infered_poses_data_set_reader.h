#pragma once

#include "rhoban_model_learning/data_set_reader.h"

namespace rhoban_model_learning
{
class InferedPosesDataSetReader : public DataSetReader
{
public:
  InferedPosesDataSetReader();

  /// Throw an error if:
  /// - none of `nb_training_images` and `nb_validation_images` is provided
  /// - The sum of `nb_training_images` and `nb_validation_tags` is higher
  ///   than the number of images with at least 'min_tags'
  virtual DataSet extractSamples(const std::string& file_path, std::default_random_engine* engine) const override;

  virtual std::string getClassName() const override;
  Json::Value toJson() const override;
  void fromJson(const Json::Value& v, const std::string& dir_name) override;

private:
  /// Number of images used for the training data set
  int nb_training_images;

  /// Number of images used for the validation data set
  int nb_validation_images;

  /// Number of tags used to infer the camera extrinsic parameters
  /// in each image.
  int nb_tags_to_infer_pose;

  /// For each training image, the number of tags used
  /// in addition to the tags to infer
  int nb_tags_per_image;

  /// Printing debug information
  bool verbose;

  /// Grid size
  int grid_height;
  int grid_width;
};

}  // namespace rhoban_model_learning

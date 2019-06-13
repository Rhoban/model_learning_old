#include "rhoban_model_learning/humanoid_models/poses_optimization_data_set_reader.h"

#include "rhoban_model_learning/humanoid_models/poses_optimization_input.h"
#include "rhoban_model_learning/humanoid_models/pose_model.h"

#include <rhoban_random/tools.h>
#include <rhoban_utils/tables/string_table.h>
#include <rhoban_utils/util.h>

#include <hl_monitoring/replay_image_provider.h>
#include "hl_monitoring/utils.h"

#include <iostream>

namespace rhoban_model_learning
{
using rhoban_utils::StringTable;

typedef PosesOptimizationDataSetReader PODSR;
typedef PosesOptimizationInput POI;

PODSR::PosesOptimizationDataSetReader() : nb_training_tags(-1), nb_validation_tags(-1), verbose(false)
{
}

DataSet PODSR::extractSamples(const std::string& file_path, std::default_random_engine* engine) const
{
  StringTable data = StringTable::buildFromFile(file_path);

  std::map<std::vector<int>, std::unique_ptr<Sample>> samples_by_corner_id_marker_id_image_id;
  std::set<std::vector<int>> keys;
  std::set<int> marker_ids;
  for (size_t row = 0; row < data.nbRows(); row++)
  {
    std::map<std::string, std::string> row_content = data.getRow(row);
    int image_id = std::stoi(row_content.at("image_id"));
    int marker_id = std::stoi(row_content.at("marker_id"));
    marker_ids.insert(marker_id);
    int corner_id = std::stoi(row_content.at("corner_id"));
    std::vector<int> key{ marker_id, corner_id, image_id };
    keys.insert(key);
    double pixel_x = std::stod(row_content.at("pixel_x"));
    double pixel_y = std::stod(row_content.at("pixel_y"));

    // camera from self
    Eigen::Affine3d camera_from_self =
        hl_monitoring::getAffineFromProtobuf(camera_from_self_poses.getCameraMetaInformation(image_id).pose());

    // camera from self
    Eigen::Affine3d camera_from_head_base =
        hl_monitoring::getAffineFromProtobuf(camera_from_head_base_poses.getCameraMetaInformation(image_id).pose());

    samples_by_corner_id_marker_id_image_id[key] = std::unique_ptr<Sample>(new Sample(
        std::unique_ptr<Input>(new POI(image_id, marker_id, corner_id, camera_from_self, camera_from_head_base)),
        Eigen::Vector2d(pixel_x, pixel_y)));
  }

  std::map<int, int> validation_corner_id_per_marker_id;
  for (auto const& marker_id : marker_ids)
  {
    int corner_id = (int)rhoban_random::getKDistinctFromN(1, 4, engine)[0];  // chose randomly a corner
    validation_corner_id_per_marker_id[marker_id] = corner_id;
  }

  std::vector<std::vector<int>> training_sample_candidates_indexes;
  std::vector<std::vector<int>> validation_sample_candidates_indexes;
  for (auto const& key : keys)
  {
    int marker_id = key[0];
    int corner_id = key[1];
    if (corner_id == validation_corner_id_per_marker_id[marker_id])
    {
      validation_sample_candidates_indexes.push_back(key);
    }
    else
    {
      training_sample_candidates_indexes.push_back(key);
    }
  }

  // Checking if enough tags
  int training_candidates_size = training_sample_candidates_indexes.size();
  if (training_candidates_size < nb_training_tags)
  {
    throw std::runtime_error(DEBUG_INFO + " not enough tags (" + std::to_string(training_candidates_size) +
                             " tags available, " + std::to_string(nb_training_tags) + " required)");
  }
  int validation_candidates_size = validation_sample_candidates_indexes.size();
  if (validation_candidates_size < nb_validation_tags)
  {
    throw std::runtime_error(DEBUG_INFO + " not enough tags (" + std::to_string(validation_candidates_size) +
                             " tags available, " + std::to_string(nb_validation_tags) + " required)");
  }

  // Separating samples
  DataSet data_set;
  //// Training
  std::vector<size_t> chosen_training_indices =
      rhoban_random::getKDistinctFromN(nb_training_tags, training_candidates_size, engine);
  for (size_t training_index : chosen_training_indices)
  {
    std::vector<int> sample_index = training_sample_candidates_indexes[training_index];
    data_set.training_set.push_back(samples_by_corner_id_marker_id_image_id[sample_index]->clone());
  }

  //// Validation
  std::vector<size_t> chosen_validation_indices =
      rhoban_random::getKDistinctFromN(nb_validation_tags, validation_candidates_size, engine);
  for (size_t validation_index : chosen_validation_indices)
  {
    std::vector<int> sample_index = validation_sample_candidates_indexes[validation_index];
    data_set.validation_set.push_back(samples_by_corner_id_marker_id_image_id[sample_index]->clone());
  }

  return data_set;
}

std::string PODSR::getClassName() const
{
  return "PosesOptimizationDataSetReader";
}

Json::Value PODSR::toJson() const
{
  Json::Value v;
  v["nb_training_tags"] = nb_training_tags;
  v["nb_validation_tags"] = nb_validation_tags;
  v["verbose"] = verbose;
  return v;
}
void PODSR::fromJson(const Json::Value& v, const std::string& dir_name)
{
  (void)dir_name;
  rhoban_utils::tryRead(v, "nb_training_tags", &nb_training_tags);
  rhoban_utils::tryRead(v, "nb_validation_tags", &nb_validation_tags);
  rhoban_utils::tryRead(v, "verbose", &verbose);
  std::string path_frames_pb;
  rhoban_utils::tryRead(v, "camera_from_self", &path_frames_pb);
  camera_from_self_poses.loadMetaInformation(path_frames_pb);
  rhoban_utils::tryRead(v, "camera_from_head_base", &path_frames_pb);
  camera_from_head_base_poses.loadMetaInformation(path_frames_pb);
}

}  // namespace rhoban_model_learning

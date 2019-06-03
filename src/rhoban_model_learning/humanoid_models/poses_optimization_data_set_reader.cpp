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

  std::vector<Sample> samples;
  for (size_t row = 0; row < data.nbRows(); row++)
  {
    std::map<std::string, std::string> row_content = data.getRow(row);
    int image_id = std::stoi(row_content.at("image_id"));
    int marker_id = std::stoi(row_content.at("marker_id"));
    double pixel_x = std::stod(row_content.at("pixel_x"));
    double pixel_y = std::stod(row_content.at("pixel_y"));

    cv::Mat r_vec;
    cv::Mat t_vec;
    hl_monitoring::pose3DToCV(replay_image_provider.getCameraMetaInformation(image_id).pose(), &r_vec, &t_vec);
    PoseModel camera_from_self;
    camera_from_self.setFromOpenCV(r_vec, t_vec);

    samples.push_back(Sample(std::unique_ptr<Input>(new POI(image_id, marker_id, camera_from_self)),
                             Eigen::Vector2d(pixel_x, pixel_y)));
  }

  size_t nb_tags = nb_validation_tags + nb_training_tags;
  if (samples.size() < nb_tags)
  {
    throw std::runtime_error(DEBUG_INFO + " not enough tags (" + std::to_string(samples.size()) + " tags available, " +
                             std::to_string(nb_training_tags + nb_validation_tags) + " required)");
  }
  // Separating samples
  DataSet data_set;
  std::vector<size_t> chosen_indices = rhoban_random::getKDistinctFromN(nb_tags, samples.size(), engine);
  std::vector<size_t> set_sizes = { (size_t)nb_training_tags, (size_t)nb_validation_tags };
  std::vector<std::vector<size_t>> samples_separation =
      rhoban_random::splitIndices(samples.size() - 1, set_sizes, engine);
  for (size_t training_idx : samples_separation[0])
  {
    data_set.training_set.push_back(samples[training_idx].clone());
  }
  for (size_t validation_idx : samples_separation[1])
  {
    data_set.validation_set.push_back(samples[validation_idx].clone());
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
  replay_image_provider.loadMetaInformation(path_frames_pb);
}

}  // namespace rhoban_model_learning

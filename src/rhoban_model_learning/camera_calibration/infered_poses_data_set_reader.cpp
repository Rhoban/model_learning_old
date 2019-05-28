#include "rhoban_model_learning/camera_calibration/infered_poses_data_set_reader.h"
#include "rhoban_model_learning/camera_calibration/infered_poses_input.h"

#include <rhoban_random/tools.h>
#include <rhoban_utils/tables/string_table.h>
#include <rhoban_utils/util.h>

#include <Eigen/Core>

#include <iostream>

namespace rhoban_model_learning
{
using rhoban_utils::StringTable;

typedef InferedPosesDataSetReader IPDSR;
typedef InferedPosesInput IPI;

IPDSR::InferedPosesDataSetReader()
  : nb_training_images(-1)
  , nb_validation_images(-1)
  , nb_tags_to_infer_pose(-1)
  , nb_tags_per_image(-1)
  , verbose(false)
  , grid_height(-1)
  , grid_width(-1)
{
}

DataSet IPDSR::extractSamples(const std::string& file_path, std::default_random_engine* engine) const
{
  StringTable data = StringTable::buildFromFile(file_path);

  std::map<int, std::vector<Eigen::Vector3d>> raw_datas_by_image;
  for (size_t row = 0; row < data.nbRows(); row++)
  {
    std::map<std::string, std::string> row_content = data.getRow(row);
    int img_id = std::stoi(row_content.at("img_id"));
    int marker_id = std::stoi(row_content.at("marker_id"));
    double img_x = std::stod(row_content.at("img_x"));
    double img_y = std::stod(row_content.at("img_y"));

    raw_datas_by_image[img_id].push_back(Eigen::Vector3d(marker_id, img_x, img_y));
  }

  // Get valid images indices
  int nb_images = nb_training_images + nb_validation_images;
  if (verbose)
  {
    std::cout << "Filtering images" << std::endl;
  }
  std::vector<int> images_indices;
  for (const auto& pair : raw_datas_by_image)
  {
    int img_id = pair.first;
    int nb_datas = pair.second.size();
    if (nb_datas >= nb_tags_to_infer_pose + nb_tags_per_image)
    {
      images_indices.push_back(pair.first);
    }
    else if (verbose)
    {
      std::cout << "\tIgnoring image " << img_id << " because it has only " << nb_datas << "(< "
                << nb_tags_to_infer_pose + nb_tags_per_image << ") valid samples" << std::endl;
    }
  }
  if (images_indices.size() < (size_t)(nb_images))
  {
    throw std::runtime_error(DEBUG_INFO + " not enough images with enough tags (" +
                             std::to_string(images_indices.size()) + " images available, " + std::to_string(nb_images) +
                             " required)");
  }

  // Choosing the images used for training and validation
  std::vector<size_t> chosen_image_indices = rhoban_random::getKDistinctFromN(nb_images, images_indices.size(), engine);

  // Choosing in each image the tags used to infer the pose
  std::map<int, std::vector<Sample>> samples_by_image;

  for (size_t id_image : chosen_image_indices)
  {
    const std::vector<Eigen::Vector3d>& raw_datas_of_image = raw_datas_by_image[images_indices[id_image]];

    std::vector<size_t> id_tags_to_infer{ 0, 5, 30, 48, 53 };
    std::vector<Eigen::Vector3d> tags_to_infer;
    for (size_t id_tag : id_tags_to_infer)
    {
      tags_to_infer.push_back(raw_datas_of_image[id_tag]);
    }
    std::sort(id_tags_to_infer.begin(), id_tags_to_infer.end());

    // We remove the tags used to infer the pose
    std::vector<int> ivec(grid_width * grid_height);
    int i = 0;
    std::iota(ivec.begin(), ivec.end(), 0);
    std::vector<int> remaining_tags;
    std::set_difference(ivec.begin(), ivec.end(), id_tags_to_infer.begin(), id_tags_to_infer.end(),
                        std::inserter(remaining_tags, remaining_tags.begin()));

    // We take nb_tags_per_image tags among the remaining tags
    std::vector<size_t> indices = rhoban_random::getKDistinctFromN(nb_tags_per_image, remaining_tags.size(), engine);

    for (size_t id_tag : indices)
    {
      Eigen::Vector3d tag = raw_datas_of_image[remaining_tags[id_tag]];
      samples_by_image[id_image].push_back(
          Sample(std::unique_ptr<Input>(new IPI(tags_to_infer, tag[0])), Eigen::Vector2d(tag[1], tag[2])));
    }
  }

  // Choosing the data set by separating the images for training and validation

  DataSet data_set;
  std::vector<size_t> set_sizes = { (size_t)nb_training_images, (size_t)nb_validation_images };
  std::vector<std::vector<size_t>> training_and_validation_image_indices_separation =
      rhoban_random::splitIndices(samples_by_image.size() - 1, set_sizes, engine);

  for (size_t idx : training_and_validation_image_indices_separation[0])
  {
    for (const Sample& sample : samples_by_image[idx])
    {
      data_set.training_set.push_back(sample.clone());
    }
  }
  for (size_t idx : training_and_validation_image_indices_separation[1])
  {
    for (const Sample& sample : samples_by_image[idx])
    {
      data_set.validation_set.push_back(sample.clone());
    }
  }

  return data_set;
}

std::string IPDSR::getClassName() const
{
  return "CalibrationDataSetReader";
}

Json::Value IPDSR::toJson() const
{
  Json::Value v;
  v["nb_training_images"] = nb_training_images;
  v["nb_validation_images"] = nb_validation_images;
  v["nb_tags_per_image"] = nb_tags_per_image;
  // v["nb_tags_to_infer_pose"] = nb_tags_to_infer_pose;
  v["grid_height"] = grid_height;
  v["grid_width"] = grid_width;
  v["verbose"] = verbose;
  return v;
}
void IPDSR::fromJson(const Json::Value& v, const std::string& dir_name)
{
  (void)dir_name;
  rhoban_utils::tryRead(v, "nb_training_images", &nb_training_images);
  rhoban_utils::tryRead(v, "nb_validation_images", &nb_validation_images);
  rhoban_utils::tryRead(v, "nb_tags_per_image", &nb_tags_per_image);
  // rhoban_utils::tryRead(v, "nb_tags_to_infer_pose", &nb_tags_to_infer_pose);
  nb_tags_to_infer_pose = 5;
  rhoban_utils::tryRead(v, "verbose", &verbose);
  rhoban_utils::tryRead(v, "grid_height", &grid_height);
  rhoban_utils::tryRead(v, "grid_width", &grid_width);

  if (5 > nb_tags_to_infer_pose)
  {
    throw std::runtime_error(DEBUG_INFO + " the number of tags to infer pose (" +
                             std::to_string(nb_tags_to_infer_pose) + " should be bigger or equal to 5.");
  }
}

}  // namespace rhoban_model_learning

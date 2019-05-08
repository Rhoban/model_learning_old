#include "rhoban_model_learning/humanoid_models/poses_optimization_model.h"

#include <rhoban_model_learning/camera_calibration/camera_model.h>
#include "rhoban_model_learning/humanoid_models/multi_poses_model.h"
#include "rhoban_model_learning/humanoid_models/vision_noise_model.h"
#include "rhoban_model_learning/tags/aruco_collection.h"

namespace rhoban_model_learning
{
typedef PosesOptimizationModel POM;

POM::PosesOptimizationModel() : CompositeModel()
{
  models["noise"] = std::unique_ptr<Model>(new VisionNoiseModel);
  models["camera"] = std::unique_ptr<Model>(new CameraModel);
  models["poses"] = std::unique_ptr<Model>(new MultiPosesModel);
  models["tags"] = std::unique_ptr<Model>(new ArucoCollection);
}

POM::PosesOptimizationModel(const PosesOptimizationModel& other) : CompositeModel(other)
{
}

double POM::getPxStddev() const
{
  return static_cast<const VisionNoiseModel&>(*models.at("noise")).px_stddev;
}

const Leph::CameraModel& POM::getCameraModel() const
{
  return static_cast<const CameraModel&>(*models.at("camera")).model;
}

const PoseModel& POM::getPose(int idx) const
{
  return static_cast<const MultiPosesModel&>(*models.at("poses")).getPose(idx);
}

const Eigen::Vector3d& POM::getTagPosition(int tag_idx) const
{
  const std::map<int, ArucoTag> markers = dynamic_cast<const TagsCollection&>(*models.at("tags")).getMarkers();
  try
  {
    return markers.at(tag_idx).marker_center;
  }
  catch (const std::out_of_range& exc)
  {
    std::ostringstream tags_oss;
    for (const auto& entry : markers)
    {
      tags_oss << entry.first << ", ";
    }
    throw std::out_of_range(DEBUG_INFO + " cannot find '" + std::to_string(tag_idx) +
                            "', available indices: " + tags_oss.str());
  }
}

std::unique_ptr<Model> POM::clone() const
{
  return std::unique_ptr<Model>(new POM(*this));
}

void POM::fromJson(const Json::Value& json_value, const std::string& dir_name)
{
  CompositeModel::fromJson(json_value, dir_name);
  // Checking content
  checkType<VisionNoiseModel>("noise");
  checkType<CameraModel>("camera");
  checkType<MultiPosesModel>("poses");
  checkType<TagsCollection>("tags");
}

std::string POM::getClassName() const
{
  return "PosesOptimizationModel";
}

}  // namespace rhoban_model_learning

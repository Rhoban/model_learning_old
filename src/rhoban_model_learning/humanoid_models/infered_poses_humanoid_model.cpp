#include "rhoban_model_learning/humanoid_models/infered_poses_humanoid_model.h"

#include "rhoban_model_learning/camera_calibration/camera_model.h"
#include "rhoban_model_learning/humanoid_models/pose_model.h"
#include "rhoban_model_learning/humanoid_models/vision_noise_model.h"

#include "rhoban_model_learning/tags/aruco_collection.h"

#include <rhoban_utils/util.h>
#include <iostream>

namespace rhoban_model_learning
{
typedef InferedPosesHumanoidModel IPHM;

IPHM::InferedPosesHumanoidModel() : CompositeModel()
{
  models["noise"] = std::unique_ptr<Model>(new VisionNoiseModel);
  models["camera"] = std::unique_ptr<Model>(new CameraModel);
  models["camera_corrected_from_camera"] = std::unique_ptr<Model>(new PoseModel);
  models["pitch_correction"] = std::unique_ptr<Model>(new PoseModel);
  models["tags"] = std::unique_ptr<Model>(new ArucoCollection);
}

IPHM::InferedPosesHumanoidModel(const InferedPosesHumanoidModel& other) : CompositeModel(other)
{
}

double IPHM::getPxStddev() const
{
  return static_cast<const VisionNoiseModel&>(*models.at("noise")).px_stddev;
}

const rhoban::CameraModel& IPHM::getCameraModel() const
{
  return static_cast<const CameraModel&>(*models.at("camera")).model;
}

const PoseModel& IPHM::getCameraCorrection() const
{
  return static_cast<const PoseModel&>(*models.at("camera_corrected_from_camera"));
}

const DoublesModel& IPHM::getPitchCorrectionModel() const
{
  return static_cast<const DoublesModel&>(*models.at("pitch_correction"));
}

std::unique_ptr<Model> IPHM::clone() const
{
  return std::unique_ptr<Model>(new IPHM(*this));
}

Eigen::Vector3d IPHM::getTagPosition(int i) const
{
  ArucoCollection tags_collection = static_cast<const ArucoCollection&>(*models.at("tags"));
  ArucoTag tag = tags_collection.getMarkers()[i];
  return tag.marker_center;
}

const Eigen::Vector3d IPHM::getCornerPosition(int tag_idx, int corner_idx) const
{
  const std::map<int, ArucoTag> markers = dynamic_cast<const TagsCollection&>(*models.at("tags")).getMarkers();
  try
  {
    return markers.at(tag_idx).getCorner(corner_idx);
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
void IPHM::fromJson(const Json::Value& v, const std::string& dir_name)
{
  CompositeModel::fromJson(v, dir_name);

  // Checking that content has been appropriately set
  checkType<VisionNoiseModel>("noise");
  checkType<CameraModel>("camera");
  checkType<PoseModel>("camera_corrected_from_camera");
  checkType<DoublesModel>("pitch_correction");
  checkType<ArucoCollection>("tags");
}

std::string IPHM::getClassName() const
{
  return "InferedPosesHumanoidModel";
}

}  // namespace rhoban_model_learning

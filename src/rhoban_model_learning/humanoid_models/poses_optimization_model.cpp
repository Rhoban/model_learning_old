#include "rhoban_model_learning/humanoid_models/poses_optimization_model.h"

#include <rhoban_model_learning/camera_calibration/camera_model.h>
#include "rhoban_model_learning/humanoid_models/vision_noise_model.h"
#include "rhoban_model_learning/basic_models/pose_2d_model.h"
#include "rhoban_model_learning/tags/aruco_collection.h"

namespace rhoban_model_learning
{
typedef PosesOptimizationModel POM;

POM::PosesOptimizationModel() : CompositeModel()
{
  models["noise"] = std::unique_ptr<Model>(new VisionNoiseModel);
  models["camera"] = std::unique_ptr<Model>(new CameraModel);
  models["camera_correction"] = std::unique_ptr<Model>(new PoseModel);
  models["head_base_correction"] = std::unique_ptr<Model>(new PoseModel);
  models["robot_2d_pose"] = std::unique_ptr<Model>(new Pose2DModel);
  models["tags"] = std::unique_ptr<Model>(new ArucoCollection);
}

POM::PosesOptimizationModel(const PosesOptimizationModel& other) : CompositeModel(other)
{
}

double POM::getPxStddev() const
{
  return static_cast<const VisionNoiseModel&>(*models.at("noise")).px_stddev;
}

const rhoban::CameraModel& POM::getCameraModel() const
{
  return static_cast<const CameraModel&>(*models.at("camera")).model;
}

const PoseModel& POM::getCameraCorrectionModel() const
{
  return static_cast<const PoseModel&>(*models.at("camera_correction"));
}

const PoseModel& POM::getHeadBaseCorrectionModel() const
{
  return static_cast<const PoseModel&>(*models.at("head_base_correction"));
}

const PoseModel POM::getRobot3DPose() const
{
  return (static_cast<const Pose2DModel&>(*models.at("robot_2d_pose"))).get3DPose();
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

const Eigen::Vector3d POM::getCornerPosition(int tag_idx, int corner_idx) const
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
  checkType<PoseModel>("camera_correction");
  checkType<PoseModel>("head_base_correction");
  checkType<TagsCollection>("tags");
}

std::string POM::getClassName() const
{
  return "PosesOptimizationModel";
}

}  // namespace rhoban_model_learning

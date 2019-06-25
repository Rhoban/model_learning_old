#include "rhoban_model_learning/log_models/log_machine_model.h"

#include "rhoban_model_learning/humanoid_models/vision_noise_model.h"
#include "rhoban_model_learning/basic_models/multi_poses_model.h"
#include "rhoban_model_learning/camera_calibration/camera_model.h"

#include <rhoban_utils/util.h>

namespace rhoban_model_learning
{
typedef LogMachineModel LMM;

LMM::LogMachineModel() : CompositeModel()
{
  models["noise"] = std::unique_ptr<Model>(new VisionNoiseModel);
  models["corrected_camera_from_camera"] = std::unique_ptr<Model>(new MultiPosesModel);
  models["camera"] = std::unique_ptr<Model>(new CameraModel);

  histories.pose("camera_from_field");
  histories.boolean("is_valid");
  histories.number("sequence_number");
  histories.number("px_std_dev");
}

LMM::LogMachineModel(const LogMachineModel& other) : CompositeModel(other)
{
  // XXX Il faut pouvoir copier les history collection
  // histories(other.histories);
}

double LMM::getPxStddev() const
{
  return static_cast<const VisionNoiseModel&>(*models.at("noise")).px_stddev;
}

const rhoban::CameraModel& LMM::getCameraModel() const
{
  return static_cast<const CameraModel&>(*models.at("camera")).model;
}

Eigen::Affine3d LMM::getCorrectedCameraFromCamera(int index) const
{
  Eigen::Affine3d res =
      static_cast<const MultiPosesModel&>(*models.at("corrected_camera_from_camera")).getPose(index).getAffine3d();
  return res;
}

Eigen::Affine3d LMM::getCorrectedCameraFromCamera(double timestamp) const
{
  bool is_valid = histories.boolean("is_valid")->interpolate(timestamp);
  if (not is_valid)
  {
    throw std::runtime_error(DEBUG_INFO + "The correction pose at timestamp " + std::to_string(timestamp) +
                             " is not valid.");
  }
  int index = std::round(histories.number("sequence_nb")->interpolate(timestamp));

  return getCorrectedCameraFromCamera(index);
}

Eigen::Affine3d LMM::getCameraFromField(double timestamp) const
{
  Eigen::Affine3d res = histories.pose("camera_from_field")->interpolate(timestamp);
  return res;
}

Eigen::Affine3d LMM::getCorrectedCameraFromField(double timestamp) const
{
  Eigen::Affine3d corrected_pose_from_pose = getCorrectedCameraFromCamera(timestamp);
  Eigen::Affine3d pose = getPose(timestamp);
  return corrected_pose_from_pose * pose;
}

std::unique_ptr<Model> LMM::clone() const
{
  return std::unique_ptr<Model>(new LMM(*this));
}

void LMM::fromJson(const Json::Value& json_value, const std::string& dir_name)
{
  CompositeModel::fromJson(json_value, dir_name);
  // Checking content
  checkType<VisionNoiseModel&>("noise");
  checkType<MultiPosesModel&>("corrected_pose_from_pose");

  std::string histories_file_path = rhoban_utils::read<std::string>(json_value, "histories_path");
  histories.loadReplays(histories_file_path);
}

std::string LMM::getClassName() const
{
  return "LogMachineModel";
}

}  // namespace rhoban_model_learning

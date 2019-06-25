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
  bool is_valid = histories.requestValues(timestamp)["is_valid:value"] == 1.0 ? true : false;
  if (not is_valid)
  {
    throw std::runtime_error(DEBUG_INFO + "The correction pose at timestamp " + std::to_string(timestamp) +
                             " is not valid.");
  }
  int index = std::round(histories.requestValues(timestamp)["sequence_nb:value"]);

  return getCorrectedCameraFromCamera(index);
}

Eigen::Affine3d LMM::getCameraFromField(double timestamp) const
{
  double tx = histories.requestValues(timestamp)["camera_from_field:tx"];
  double ty = histories.requestValues(timestamp)["camera_from_field:ty"];
  double tz = histories.requestValues(timestamp)["camera_from_field:tz"];

  double qx = histories.requestValues(timestamp)["camera_from_field:qx"];
  double qy = histories.requestValues(timestamp)["camera_from_field:qy"];
  double qz = histories.requestValues(timestamp)["camera_from_field:qz"];
  double qw = histories.requestValues(timestamp)["camera_from_field:qw"];

  Eigen::Affine3d res;
  res.translation() = Eigen::Vector3d(tx, ty, tz);
  res.rotation() = Eigen::Quaterniond(qw, qx, qy, qz);

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

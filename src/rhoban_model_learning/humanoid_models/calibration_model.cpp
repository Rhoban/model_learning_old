#include "rhoban_model_learning/humanoid_models/calibration_model.h"

#include <rhoban_model_learning/camera_calibration/camera_model.h>
#include "rhoban_model_learning/humanoid_models/rotation_model.h"
#include "rhoban_model_learning/humanoid_models/vision_noise_model.h"

#include <rhoban_utils/util.h>

namespace rhoban_model_learning
{
typedef CalibrationModel CM;

CM::CalibrationModel() : CompositeModel()
{
  models["camera"] = std::unique_ptr<Model>(new CameraModel);
  models["camera_corrected_from_camera"] = std::unique_ptr<Model>(new PoseModel);
  models["head_base_corrected_from_head_base"] = std::unique_ptr<Model>(new PoseModel);
  models["noise"] = std::unique_ptr<Model>(new VisionNoiseModel);
}

CM::CalibrationModel(const CalibrationModel& other) : CompositeModel(other)
{
}

double CM::getPxStddev() const
{
  return static_cast<const VisionNoiseModel&>(*models.at("noise")).px_stddev;
}

const Eigen::Affine3d CM::getCameraFromSelfAfterCorrection(Eigen::Affine3d camera_from_self,
                                                           Eigen::Affine3d head_base_from_camera) const
{
  Eigen::Affine3d camera_corrected_from_camera = getCameraCorrectedFromCamera();
  Eigen::Affine3d head_base_corrected_from_head_base = getHeadBaseCorrectedFromHeadBase();
  Eigen::Affine3d camera_from_head_base = head_base_from_camera.inverse();

  return camera_corrected_from_camera * camera_from_head_base * head_base_corrected_from_head_base *
         head_base_from_camera * camera_from_self;
}

const Eigen::Affine3d CM::getCameraCorrectedFromCamera() const
{
  return (static_cast<const PoseModel&>(*models.at("camera_corrected_from_camera"))).getAffine3d();
}

const Eigen::Affine3d CM::getHeadBaseCorrectedFromHeadBase() const
{
  return (static_cast<const PoseModel&>(*models.at("head_base_corrected_from_head_base"))).getAffine3d();
}

const rhoban::CameraModel& CM::getCameraModel() const
{
  return static_cast<const CameraModel&>(*models.at("camera")).model;
}

void CM::setCameraFromSelf(PoseModel pose)
{
  camera_from_self = pose;
}
void CM::setCameraFromHeadBase(PoseModel pose)
{
  camera_from_head_base = pose;
}

std::unique_ptr<Model> CM::clone() const
{
  return std::unique_ptr<Model>(new CM(*this));
}
void CM::fromJson(const Json::Value& v, const std::string& dir_name)
{
  CompositeModel::fromJson(v, dir_name);
  // Checking that content has been appropriately set
  try
  {
    dynamic_cast<const CameraModel&>(*models.at("camera"));
  }
  catch (const std::bad_cast& e)
  {
    throw std::runtime_error(DEBUG_INFO + " invalid type for 'camera'");
  }
  try
  {
    dynamic_cast<const VisionNoiseModel&>(*models.at("noise"));
  }
  catch (const std::bad_cast& e)
  {
    throw std::runtime_error(DEBUG_INFO + " invalid type for 'noise'");
  }
  try
  {
    dynamic_cast<const PoseModel&>(*models.at("head_base_corrected_from_head_base"));
  }
  catch (const std::bad_cast& e)
  {
    throw std::runtime_error(DEBUG_INFO + " invalid type for 'head_base_corrected_from_head_base'");
  }
  try
  {
    dynamic_cast<const PoseModel&>(*models.at("camera_corrected_from_camera"));
  }
  catch (const std::bad_cast& e)
  {
    throw std::runtime_error(DEBUG_INFO + " invalid type for 'camera_corrected_from_camera'");
  }
}

std::string CM::getClassName() const
{
  return "CalibrationModel";
}

}  // namespace rhoban_model_learning

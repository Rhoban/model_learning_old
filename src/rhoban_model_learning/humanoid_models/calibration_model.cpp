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
  models["camOffset"] = std::unique_ptr<Model>(new RotationModel);
  models["imuOffset"] = std::unique_ptr<Model>(new RotationModel);
  models["neckOffset"] = std::unique_ptr<Model>(new RotationModel);
  models["noise"] = std::unique_ptr<Model>(new VisionNoiseModel);
}

CM::CalibrationModel(const CalibrationModel& other) : CompositeModel(other)
{
}

double CM::getPxStddev() const
{
  return static_cast<const VisionNoiseModel&>(*models.at("noise")).px_stddev;
}

const RotationModel& CM::getRotationModel(const std::string& name) const
{
  return static_cast<const RotationModel&>(*models.at(name));
}

Eigen::Vector3d CM::getCameraOffsetsRad() const
{
  return M_PI / 180 * getRotationModel("camOffset").getRPY();
}

Eigen::Vector3d CM::getImuOffsetsRad() const
{
  return M_PI / 180 * getRotationModel("imuOffset").getRPY();
}

Eigen::Vector3d CM::getNeckOffsetsRad() const
{
  return M_PI / 180 * getRotationModel("neckOffset").getRPY();
}

const rhoban::CameraModel& CM::getCameraModel() const
{
  return static_cast<const CameraModel&>(*models.at("camera")).model;
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
    dynamic_cast<const RotationModel&>(*models.at("imuOffset"));
  }
  catch (const std::bad_cast& e)
  {
    throw std::runtime_error(DEBUG_INFO + " invalid type for 'imuOffset'");
  }
  try
  {
    dynamic_cast<const RotationModel&>(*models.at("camOffset"));
  }
  catch (const std::bad_cast& e)
  {
    throw std::runtime_error(DEBUG_INFO + " invalid type for 'camOffset'");
  }
  try
  {
    dynamic_cast<const RotationModel&>(*models.at("neckOffset"));
  }
  catch (const std::bad_cast& e)
  {
    throw std::runtime_error(DEBUG_INFO + " invalid type for 'neckOffset'");
  }
}

std::string CM::getClassName() const
{
  return "CalibrationModel";
}

}  // namespace rhoban_model_learning

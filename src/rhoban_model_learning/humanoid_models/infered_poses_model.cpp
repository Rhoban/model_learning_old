#include "rhoban_model_learning/humanoid_models/infered_poses_model.h"

#include <rhoban_model_learning/camera_calibration/camera_model.h>
#include "rhoban_model_learning/humanoid_models/multi_poses_model.h"
#include "rhoban_model_learning/humanoid_models/vision_noise_model.h"

#include <rhoban_utils/util.h>
#include <iostream>

namespace rhoban_model_learning
{
typedef InferedPosesModel IPM;

IPM::InferedPosesModel() : CompositeModel()
{
  models["noise"] = std::unique_ptr<Model>(new VisionNoiseModel);
  models["camera"] = std::unique_ptr<Model>(new CameraModel);
}

IPM::InferedPosesModel(const InferedPosesModel& other) : CompositeModel(other)
{
  for (auto& el : other.markers)
  {
    markers[el.first] = el.second;
  }
}

double IPM::getPxStddev() const
{
  return static_cast<const VisionNoiseModel&>(*models.at("noise")).px_stddev;
}

const rhoban::CameraModel& IPM::getCameraModel() const
{
  return static_cast<const CameraModel&>(*models.at("camera")).model;
}

std::unique_ptr<Model> IPM::clone() const
{
  return std::unique_ptr<Model>(new IPM(*this));
}

Eigen::Vector3d IPM::getTagPosition(int i) const
{
  return markers.at(i);
}

void IPM::fromJson(const Json::Value& v, const std::string& dir_name)
{
  CompositeModel::fromJson(v, dir_name);

  // Checking that content has been appropriately set
  checkType<VisionNoiseModel>("noise");
  checkType<CameraModel>("camera");

  markers.loadFile(rhoban_utils::read<std::string>(v, "rel_tag_path"));
  std::cout << "Markers loaded. There are " << markers.size() << " markers." << std::endl;
}

std::string IPM::getClassName() const
{
  return "InferedPosesModel";
}

}  // namespace rhoban_model_learning

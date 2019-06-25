#pragma once

#include "rhoban_model_learning/composite_model.h"
#include "rhoban_model_learning/basic_models/multi_poses_model.h"
#include "rhoban_model_learning/camera_calibration/marker_collection.h"

#include "robot_model/camera_model.h"
#include <Eigen/Core>

namespace rhoban_model_learning
{
class InferedPosesModel : public CompositeModel
{
public:
  InferedPosesModel();
  InferedPosesModel(const InferedPosesModel& other);

  double getPxStddev() const;

  const rhoban::CameraModel& getCameraModel() const;

  virtual std::unique_ptr<Model> clone() const;

  Eigen::Vector3d getTagPosition(int i) const;

  void fromJson(const Json::Value& json_value, const std::string& dir_name) override;
  std::string getClassName() const;

protected:
  MarkerCollection markers;
};

}  // namespace rhoban_model_learning

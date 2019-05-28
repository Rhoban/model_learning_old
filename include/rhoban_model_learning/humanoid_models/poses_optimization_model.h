#pragma once

#include "rhoban_model_learning/composite_model.h"
#include "rhoban_model_learning/humanoid_models/pose_model.h"

#include "robot_model/camera_model.h"

#include <hl_monitoring/replay_image_provider.h>

namespace rhoban_model_learning
{
class PosesOptimizationModel : public CompositeModel
{
public:
  PosesOptimizationModel();
  PosesOptimizationModel(const PosesOptimizationModel& other);

  double getPxStddev() const;

  const rhoban::CameraModel& getCameraModel() const;
  const PoseModel& getCameraCorrectionModel() const;
  const PoseModel getRobot3DPose() const;

  const Eigen::Vector3d& getTagPosition(int tag_idx) const;

  virtual std::unique_ptr<Model> clone() const;

  void fromJson(const Json::Value& json_value, const std::string& dir_name) override;
  std::string getClassName() const;

  hl_monitoring::ReplayImageProvider log_provider;
};

}  // namespace rhoban_model_learning

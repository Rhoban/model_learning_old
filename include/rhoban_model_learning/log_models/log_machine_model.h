#pragma once
#include "rhoban_model_learning/composite_model.h"

#include "rhoban_utils/history/history.h"
#include "rhoban_model_learning/camera_calibration/camera_model.h"

#include "robot_model/camera_model.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace rhoban_model_learning
{
class LogMachineModel : public CompositeModel
{
public:
  LogMachineModel();
  LogMachineModel(const LogMachineModel& other);
  LogMachineModel(const rhoban_utils::HistoryCollection& _histories, std::vector<double> _sequences_timestamps);

  virtual std::unique_ptr<Model> clone() const;

  double getPxStddev() const;

  const rhoban::CameraModel& getCameraModel() const;

  int getIndex(double timestamp) const;

  Eigen::Affine3d getCorrectedCameraFromCamera(int index) const;
  Eigen::Affine3d getCorrectedCameraFromCamera(double timestamp) const;

  Eigen::Affine3d getCorrectedCameraFromField(double timestamp) const;
  Eigen::Affine3d getCameraFromField(double timestamp) const;

  void fromJson(const Json::Value& json_value, const std::string& dir_name) override;
  std::string getClassName() const;

  // Histories
  //// "poses" : PoseModel history
  //// "is_valid" : bool history
  //// "px_std_dev" : double history
  rhoban_utils::HistoryCollection histories;

private:
  // sequences start timestamps
  // seq_1_start, seq_2_start, seq_3_start, ...
  std::vector<double> sequences_timestamps;
};

}  // namespace rhoban_model_learning

#pragma once

#include "rhoban_model_learning/composite_model.h"
#include "rhoban_model_learning/humanoid_models/rotation_model.h"
#include "rhoban_model_learning/basic_models/pose_model.h"

#include "robot_model/camera_model.h"

namespace rhoban_model_learning
{
/// This class is used to represent the correction of the model based on vision
/// informations. It can be used to learn static offset in various locations.
///
/// Inputs are composed of:
/// - Position of all the DOF and the IMU
/// - Expected position of the data in the left_foot referential
///
/// Observations are positions of the object in the image (x,y)
class CalibrationModel : public CompositeModel
{
public:
  CalibrationModel();
  CalibrationModel(const CalibrationModel& other);

  double getPxStddev() const;

  /// Return the rotation Model corresponding to the given name
  /// throws error if name is not valid
  const RotationModel& getRotationModel(const std::string& name) const;

  /// Interface for ModelService
  const Eigen::Affine3d getCameraFromSelfAfterCorrection(Eigen::Affine3d camera_from_self,
                                                         Eigen::Affine3d head_base_from_camera) const;

  /// Camera pose correction model
  const Eigen::Affine3d getCameraCorrectedFromCamera() const;
  /// Head base pose correction model
  const Eigen::Affine3d getHeadBaseCorrectedFromHeadBase() const;
  /* /// IMU pose correction model */
  /* const Eigen::Matrix3d getImuCorrectedFromImu() const; */

  const rhoban::CameraModel& getCameraModel() const;

  virtual std::unique_ptr<Model> clone() const;

  void fromJson(const Json::Value& json_value, const std::string& dir_name) override;
  std::string getClassName() const;

  void setCameraFromSelf(PoseModel pose);
  void setCameraFromHeadBase(PoseModel pose);

  PoseModel camera_from_self;
  PoseModel camera_from_head_base;
};

}  // namespace rhoban_model_learning

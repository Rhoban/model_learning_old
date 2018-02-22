#pragma once

#include "rhoban_model_learning/model.h"
#include "rhoban_model_learning/input_reader.h"

#include "Model/HumanoidModel.hpp"
#include "Model/NamesModel.h"

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
class VisionCorrectionModel : public Model {
public:

  class VisionInput : public Input {
  public:
    VisionInput();
    VisionInput(const Leph::VectorLabel & data);
    VisionInput(const VisionInput & other);

    virtual std::unique_ptr<Input> clone() const override;

    /// Contains all DOF + IMU + pixel position
    Leph::VectorLabel data;
  };

  class VisionInputReader : public InputReader {
  public:
    VisionInputReader();

    virtual DataSet extractSamples(const std::string & file_path,
                                   std::default_random_engine * engine) const override;

    virtual std::string getClassName() const override;
    Json::Value toJson() const override;
    void fromJson(const Json::Value & v, const std::string & dir_name) override;
  private:
    /// Ratio of logs used for validation. Value in [0,1]
    double validation_ratio;
  };


  VisionCorrectionModel();

  virtual Eigen::VectorXd getParameters() const override;
  virtual void setParameters(const Eigen::VectorXd & new_params) override;
  virtual std::vector<std::string> getParametersNames() const override;
  virtual Eigen::VectorXi getObservationsCircularity() const override;

  virtual Eigen::VectorXd
  predictObservation(const Input & input,
                     std::default_random_engine * engine) const;

  /// Density
  virtual double computeLogLikelihood(const Sample & sample,
                                      std::default_random_engine * engine) const override;

  virtual std::unique_ptr<Model> clone() const;

  Json::Value toJson() const override;
  void fromJson(const Json::Value & v, const std::string & dir_name) override;
  std::string getClassName() const;

private:
  /// The observation standard deviation in pixels
  double px_stddev;

  /// Offset at camera fixation (roll, pitch, yaw)
  Eigen::Vector3d cam_offset;
  /// Offset between IMU orientation and trunk orientation (roll, pitch, yaw)
  Eigen::Vector3d imu_offset;
  /// Offset in neck (roll, pitch, yaw)
  Eigen::Vector3d neck_offset;

  // The width and height aperture of the camera
  Leph::CameraParameters camera_parameters;
};

}

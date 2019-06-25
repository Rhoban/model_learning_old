#include "rhoban_model_learning/log_models/log_machine_predictor.h"
#include "rhoban_model_learning/log_models/log_machine_input.h"
#include "rhoban_model_learning/log_models/log_machine_model.h"

#include "rhoban_random/multivariate_gaussian.h"
#include "rhoban_random/tools.h"

#include <rhoban_utils/angle.h>
#include <rhoban_utils/util.h>

#include <iostream>

namespace rhoban_model_learning
{
typedef LogMachinePredictor LMP;

LMP::LogMachinePredictor()
{
}

Eigen::VectorXd LMP::predictObservation(const Input& raw_input, const Model& raw_model,
                                        std::default_random_engine* engine) const
{
  const LogMachineInput& input = dynamic_cast<const LogMachineInput&>(raw_input);
  const LogMachineModel& log_machine_model = dynamic_cast<const LogMachineModel&>(raw_model);

  double timestamp = input.timestamp;

  // In field
  Eigen::Vector3d position_in_field = input.label_position;

  // In corrected camera
  Eigen::Affine3d corrected_camera_from_field = log_machine_model.getCorrectedCameraFromField(timestamp);
  Eigen::Vector3d position_in_corrected_camera = corrected_camera_from_field * position_in_field;

  // In image
  Eigen::Vector2d pixel;
  try
  {
    pixel = cv2Eigen(log_machine_model.getCameraModel().getImgFromObject(eigen2CV(position_in_corrected_camera)));
  }
  catch (const std::runtime_error& exc)
  {
    // std::cout << "Marker " << input.aruco_id << " corner " << input.corner_id
    //           << " pos in world: " << marker_pose.transpose() << std::endl;
    // std::cout << "Marker " << input.aruco_id << " corner " << input.corner_id
    //           << " pos in self: " << marker_pose_in_self.transpose() << std::endl;
    // std::cout << "Marker " << input.aruco_id << " corner " << input.corner_id
    //           << " pos in camera: " << marker_pose_in_camera_after_correction.transpose() << std::endl;
    // std::cout << "Camera pose in self before correction " << std::endl << camera_from_self.matrix() << std::endl;
    // std::cout << "Camera pose in self after correction " << std::endl
    //           << camera_from_self_after_correction.matrix() << std::endl;
    // std::cerr << "error: " << exc.what() << std::endl;
    // exit(EXIT_FAILURE);
  }

  // Add noise if required
  if (engine != nullptr)
  {
    std::normal_distribution<double> observation_noise(0, log_machine_model.getPxStddev());
    pixel(0) += observation_noise(*engine);
    pixel(1) += observation_noise(*engine);
  }
  return pixel;
}

double LMP::computeLogLikelihood(const Sample& sample, const Model& raw_model, std::default_random_engine* engine) const
{
  const LogMachineModel& log_machine_model = dynamic_cast<const LogMachineModel&>(raw_model);

  (void)engine;
  Eigen::Vector2d prediction = predictObservation(sample.getInput(), log_machine_model, nullptr);
  Eigen::Vector2d observation = sample.getObservation();

  double px_stddev = log_machine_model.getPxStddev();
  double px_var = px_stddev * px_stddev;
  Eigen::MatrixXd covar(2, 2);
  covar << px_var, 0, 0, px_var;
  rhoban_random::MultivariateGaussian expected_distribution(prediction, covar);

  return expected_distribution.getLogLikelihood(observation);
}

Eigen::VectorXi LMP::getObservationsCircularity() const
{
  return Eigen::VectorXi::Zero(2);
}

std::string LMP::getClassName() const
{
  return "LogMachinePredictor";
}

std::unique_ptr<Predictor> LogMachinePredictor::clone() const
{
  return std::unique_ptr<Predictor>(new LogMachinePredictor());
}

}  // namespace rhoban_model_learning

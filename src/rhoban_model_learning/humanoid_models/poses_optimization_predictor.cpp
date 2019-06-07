#include "rhoban_model_learning/humanoid_models/poses_optimization_predictor.h"

#include "rhoban_model_learning/humanoid_models/poses_optimization_input.h"
#include "rhoban_model_learning/humanoid_models/poses_optimization_model.h"

#include "rhoban_random/multivariate_gaussian.h"
#include "rhoban_random/tools.h"

#include <rhoban_utils/angle.h>
#include <rhoban_utils/util.h>

#include <iostream>

namespace rhoban_model_learning
{
typedef PosesOptimizationPredictor POP;
POP::PosesOptimizationPredictor()
{
}

Eigen::VectorXd POP::predictObservation(const Input& raw_input, const Model& raw_model,
                                        std::default_random_engine* engine) const
{
  // Can generate bad_cast error
  const PosesOptimizationInput& input = dynamic_cast<const PosesOptimizationInput&>(raw_input);
  const PosesOptimizationModel& model = dynamic_cast<const PosesOptimizationModel&>(raw_model);

  // First: get marker pose in world frame
  Eigen::Vector3d marker_pose = model.getCornerPosition(input.aruco_id, input.corner_id);

  // Then get marker pose in camera frame (world -> self -> camera)
  PoseModel robot_pose = model.getRobot3DPose();
  Eigen::Vector3d marker_pose_in_self = robot_pose.getPosInSelf(marker_pose);

  PoseModel camera_from_self = input.camera_from_self;
  Eigen::Vector3d marker_pose_in_camera = camera_from_self.getPosFromSelf(marker_pose_in_self);

  // Apply neck pitch correction
  Eigen::Vector3d camera_to_pitch_axis(0, -0.0215, -0.064);
  double angle_rad = rhoban_utils::deg2rad(model.getPitchCorrectionModel().getParameter(0));
  Eigen::AngleAxisd r(angle_rad, Eigen::Vector3d::UnitY());
  Eigen::Vector3d tmp_marker_pose_in_camera_with_pitch_corrected = marker_pose_in_camera - camera_to_pitch_axis;
  Eigen::Vector3d marker_pose_in_camera_with_pitch_corrected =
      r * tmp_marker_pose_in_camera_with_pitch_corrected + camera_to_pitch_axis;

  // Apply camera correction
  PoseModel camera_correction = model.getCameraCorrectionModel();
  Eigen::Vector3d marker_pose_in_camera_with_pitch_and_camera_corrected =
      camera_correction.getPosInSelf(marker_pose_in_camera_with_pitch_corrected);

  // std::cout << "Marker " << input.aruco_id << " pos in self: " << marker_pose_in_self.transpose() << std::endl;
  // std::cout << "Marker " << input.aruco_id << " pos in camera: " << marker_pose_in_camera.transpose() << std::endl;
  Eigen::Vector2d pixel = cv2Eigen(
      model.getCameraModel().getImgFromObject(eigen2CV(marker_pose_in_camera_with_pitch_and_camera_corrected)));

  // Add noise if required
  if (engine != nullptr)
  {
    std::normal_distribution<double> observation_noise(0, model.getPxStddev());
    pixel(0) += observation_noise(*engine);
    pixel(1) += observation_noise(*engine);
  }
  return pixel;
}

double POP::computeLogLikelihood(const Sample& sample, const Model& raw_model, std::default_random_engine* engine) const
{
  const PosesOptimizationModel& model = dynamic_cast<const PosesOptimizationModel&>(raw_model);

  (void)engine;
  Eigen::Vector2d prediction = predictObservation(sample.getInput(), model, nullptr);
  Eigen::Vector2d observation = sample.getObservation();

  double px_stddev = model.getPxStddev();
  double px_var = px_stddev * px_stddev;
  Eigen::MatrixXd covar(2, 2);
  covar << px_var, 0, 0, px_var;
  rhoban_random::MultivariateGaussian expected_distribution(prediction, covar);

  return expected_distribution.getLogLikelihood(observation);
}

Eigen::VectorXi POP::getObservationsCircularity() const
{
  return Eigen::VectorXi::Zero(2);
}

std::string POP::getClassName() const
{
  return "PosesOptimizationPredictor";
}

void POP::exportPredictionsToCSV(const Model& raw_model, const SampleVector& sample_vector, const std::string& filename,
                                 char separator) const
{
  rhoban_utils::CSV* csv = new rhoban_utils::CSV();
  csv->open(filename, separator);

  const PosesOptimizationModel& model = dynamic_cast<const PosesOptimizationModel&>(raw_model);

  for (const auto& sample : sample_vector)
  {
    const PosesOptimizationInput& input = dynamic_cast<const PosesOptimizationInput&>(sample->getInput());
    Eigen::Vector2d prediction = predictObservation(input, model, nullptr);
    Eigen::Vector2d observation = sample->getObservation();
    csv->push("marker_id", input.aruco_id);
    csv->push("obs_x", observation.x());
    csv->push("obs_y", observation.y());
    csv->push("pred_x", prediction.x());
    csv->push("pred_y", prediction.y());
    csv->newLine();
  }
  csv->close();
}

std::unique_ptr<Predictor> PosesOptimizationPredictor::clone() const
{
  return std::unique_ptr<Predictor>(new PosesOptimizationPredictor());
}

}  // namespace rhoban_model_learning

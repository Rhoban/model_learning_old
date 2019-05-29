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
  // First: get marker position in world and pose
  Eigen::Vector3d marker_pose = model.getTagPosition(input.aruco_id);
  PoseModel robot_pose = model.getRobot3DPose();
  Eigen::Vector3d marker_pose_in_self = robot_pose.getPosInSelf(marker_pose);
  PoseModel camera_from_self = input.camera_from_field;
  Eigen::Vector3d marker_pose_in_camera = camera_from_self.getPosInSelf(marker_pose_in_self);

  std::cout << "Marker pos in self: " << marker_pose_in_self.transpose() << std::endl;
  std::cout << "Marker pos in camera: " << marker_pose_in_camera.transpose() << std::endl;
  Eigen::Vector2d pixel = cv2Eigen(model.getCameraModel().getImgFromObject(eigen2CV(marker_pose_in_camera)));

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

}  // namespace rhoban_model_learning

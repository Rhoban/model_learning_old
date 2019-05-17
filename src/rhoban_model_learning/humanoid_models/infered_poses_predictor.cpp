#include "rhoban_model_learning/humanoid_models/infered_poses_predictor.h"

#include "rhoban_model_learning/humanoid_models/infered_poses_input.h"
#include "rhoban_model_learning/humanoid_models/infered_poses_model.h"

#include "rhoban_random/multivariate_gaussian.h"
#include "rhoban_random/tools.h"

#include <rhoban_utils/angle.h>
#include <rhoban_utils/util.h>
#include <robot_model/camera_model.h>

#include <opencv2/core/eigen.hpp>
#include "opencv2/calib3d/calib3d.hpp"

namespace rhoban_model_learning
{
typedef InferedPosesPredictor IPP;
IPP::InferedPosesPredictor()
{
}

Eigen::VectorXd IPP::predictObservation(const Input& raw_input, const Model& raw_model,
                                        std::default_random_engine* engine) const
{
  // Can generate bad_cast error
  const InferedPosesInput& input = dynamic_cast<const InferedPosesInput&>(raw_input);
  const InferedPosesModel& model = dynamic_cast<const InferedPosesModel&>(raw_model);

  // First: infer matrix extrinsic parameters in world
  cv::Mat cameraMatrix = model.getCameraModel().getCameraMatrix();
  cv::Mat cameraDistortionCoeffs = model.getCameraModel().getDistortionCoeffs();

  std::vector<cv::Point3f> worldCoordinates;
  std::vector<cv::Point2f> pixelCoordinates;
  for (const Eigen::Vector3d& tag_to_infer : input.tags_to_infer)
  {
    pixelCoordinates.push_back(cv::Point2f(tag_to_infer(1), tag_to_infer(2)));
    worldCoordinates.push_back(eigen2CV(model.getTagPosition(tag_to_infer(0))));
  }

  cv::Mat t_vec;
  cv::Mat r_vec;
  cv::solvePnP(worldCoordinates, pixelCoordinates, cameraMatrix, cameraDistortionCoeffs, r_vec, t_vec);

  std::vector<cv::Point3d> marker_pos_world_container{ eigen2CV(model.getTagPosition(input.aruco_id)) };
  std::vector<cv::Point2d> pixel_container{};
  cv::projectPoints(marker_pos_world_container, r_vec, t_vec, cameraMatrix, cameraDistortionCoeffs, pixel_container);

  Eigen::Vector2d pixel = cv2Eigen(pixel_container[0]);
  // Add noise if required
  if (engine != nullptr)
  {
    std::normal_distribution<double> observation_noise(0, model.getPxStddev());
    pixel(0) += observation_noise(*engine);
    pixel(1) += observation_noise(*engine);
  }
  return pixel;
}

double IPP::computeLogLikelihood(const Sample& sample, const Model& raw_model, std::default_random_engine* engine) const
{
  const InferedPosesModel& model = dynamic_cast<const InferedPosesModel&>(raw_model);

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

Eigen::VectorXi IPP::getObservationsCircularity() const
{
  return Eigen::VectorXi::Zero(2);
}

std::string IPP::getClassName() const
{
  return "InferedPosesPredictor";
}

}  // namespace rhoban_model_learning

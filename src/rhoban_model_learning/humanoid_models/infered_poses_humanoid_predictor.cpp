#include "rhoban_model_learning/humanoid_models/infered_poses_humanoid_predictor.h"
#include "rhoban_model_learning/humanoid_models/infered_poses_humanoid_input.h"
#include "rhoban_model_learning/humanoid_models/infered_poses_humanoid_model.h"

#include "rhoban_random/multivariate_gaussian.h"
#include "rhoban_random/tools.h"

#include <rhoban_utils/angle.h>
#include <rhoban_utils/util.h>
#include "rhoban_utils/logging/csv.h"
#include <robot_model/camera_model.h>

#include <opencv2/core/eigen.hpp>
#include "opencv2/calib3d/calib3d.hpp"

namespace rhoban_model_learning
{
typedef InferedPosesHumanoidPredictor IPHP;
IPHP::InferedPosesHumanoidPredictor()
{
}

Eigen::VectorXd IPHP::predictObservation(const Input& raw_input, const Model& raw_model,
                                         std::default_random_engine* engine) const
{
  // Can generate bad_cast error
  const InferedPosesHumanoidInput& input = dynamic_cast<const InferedPosesHumanoidInput&>(raw_input);
  const InferedPosesHumanoidModel& model = dynamic_cast<const InferedPosesHumanoidModel&>(raw_model);

  // First we compute the position of the sheet in world
  cv::Mat cameraMatrix = model.getCameraModel().getCameraMatrix();
  cv::Mat cameraDistortionCoeffs = model.getCameraModel().getDistortionCoeffs();

  std::vector<cv::Point3f> worldCoordinates;
  std::vector<cv::Point2f> pixelCoordinates;
  for (const Eigen::Vector3d& tag_to_infer : input.tags_to_infer_sheet_world_pos)
  {
    pixelCoordinates.push_back(cv::Point2f(tag_to_infer(1), tag_to_infer(2)));
    worldCoordinates.push_back(eigen2CV(model.getTagPosition(tag_to_infer(0))));
  }

  // t_vec and r_vec brings points from the frame describing tags coordinates to
  // the frame of the camera.
  cv::Mat r_vec;
  cv::Mat t_vec;
  cv::solvePnP(worldCoordinates, pixelCoordinates, cameraMatrix, cameraDistortionCoeffs, r_vec, t_vec);

  PoseModel camera_pose_in_sheet;
  camera_pose_in_sheet.setFromOpenCV(r_vec, t_vec);
  //

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

double IPHP::computeLogLikelihood(const Sample& sample, const Model& raw_model,
                                  std::default_random_engine* engine) const
{
  const InferedPosesHumanoidModel& model = dynamic_cast<const InferedPosesHumanoidModel&>(raw_model);

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

std::string IPHP::getClassName() const
{
  return "InferedPosesHumanoidPredictor";
}

void IPHP::exportPredictionsToCSV(const Model& raw_model, const SampleVector& sample_vector,
                                  const std::string& filename, char separator) const
{
  rhoban_utils::CSV* csv = new rhoban_utils::CSV();
  csv->open(filename, separator);

  const InferedPosesHumanoidModel& model = dynamic_cast<const InferedPosesHumanoidModel&>(raw_model);

  for (const auto& sample : sample_vector)
  {
    const InferedPosesHumanoidInput& input = dynamic_cast<const InferedPosesHumanoidInput&>(sample->getInput());
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

}  // namespace rhoban_model_learning

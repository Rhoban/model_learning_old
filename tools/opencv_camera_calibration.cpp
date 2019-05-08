#include <rhoban_model_learning/camera_calibration/camera_model.h>
#include <rhoban_model_learning/camera_calibration/marker_collection.h>
#include <rhoban_model_learning/camera_calibration/marker_seen_collection.h>
#include <rhoban_random/tools.h>
#include <rhoban_utils/util.h>

#include <tclap/CmdLine.h>
#include <opencv2/calib3d.hpp>


using namespace rhoban_model_learning;
using namespace TCLAP;

CameraModel calibrateCamera(const MarkerCollection& markers, const MarkerSeenCollection& markers_seen, size_t nb_images,
                            const cv::Size& img_size, int calibration_flags)
{
  // Preliminary: check validity
  if (markers_seen.size() < nb_images)
  {
    throw std::runtime_error(DEBUG_INFO + "Not enough images available : requested=" + std::to_string(nb_images) +
                             ", received=" + std::to_string(markers_seen.size()));
  }
  // 1. Retrieve the id of imgs used for calibration
  std::vector<int> images_indices;
  for (const auto& entry : markers_seen)
  {
    images_indices.push_back(entry.first);
  }
  std::default_random_engine engine = rhoban_random::getRandomEngine();
  std::shuffle(images_indices.begin(), images_indices.end(), engine);
  // 2. Extract points from both markers and seen markers
  std::vector<std::vector<cv::Point3f>> obj_points;
  std::vector<std::vector<cv::Point2f>> img_points;
  for (size_t idx = 0; idx < nb_images; idx++)
  {
    int image_id = images_indices[idx];
    std::vector<cv::Point2f> tmp_img_points;
    std::vector<cv::Point3f> tmp_obj_points;
    for (const auto& entry: markers_seen.at(image_id))
    {
      int marker_id = entry.first;
      Eigen::Vector2d img_point = entry.second;
      Eigen::Vector3d obj_point = markers.at(marker_id);
      tmp_img_points.push_back(cv::Point2f(img_point.x(), img_point.y()));
      tmp_obj_points.push_back(cv::Point3f(obj_point.x(), obj_point.y(), obj_point.z()));
    }
    obj_points.push_back(tmp_obj_points);
    img_points.push_back(tmp_img_points);
  }
  // 3. Perform opencv calibration
  cv::Mat camera_matrix;
  cv::Mat distortion_coeffs;
  std::vector<cv::Mat> rvecs, tvecs;

  std::cout << "img_size: " << img_size << std::endl;

  double error = cv::calibrateCamera(obj_points, img_points, img_size, camera_matrix, distortion_coeffs, rvecs, tvecs,
                                     calibration_flags);

  std::cout << "Error: " << error << std::endl;
  std::cout << "Camera Matrix: " << camera_matrix << std::endl;
  std::cout << "Distortion Coeffs: " << distortion_coeffs << std::endl;

  // 4. Convert from opencv to rhoban_model_learning::CameraModel
  return CameraModel(Leph::CameraModel(camera_matrix, distortion_coeffs, img_size));
}

int main(int argc, char** argv)
{
  try
  {
    CmdLine cmd("Calibrate a camera based on two files, one containing geometry of the markers and the other one "
                "containing the positions of the marker in images",
                ' ', "0.1");

    UnlabeledValueArg<std::string> markers_arg("markers_paths", "path to file containing geometry of the markers", true,
                                               "markers.csv", "markers_path", cmd);
    UnlabeledValueArg<std::string> markers_seen_arg("markers_seen_paths",
                                                    "path to file containing result of markers detection", true,
                                                    "markers_seen.csv", "markers_seen_path", cmd);
    TCLAP::ValueArg<int> nb_images_arg("n", "nb-images", "Number of images used for training", false, 20, "int", cmd);
    TCLAP::ValueArg<int> cols_arg("c", "cols", "Width of the images used", false, 640, "nb_cols", cmd);
    TCLAP::ValueArg<int> rows_arg("r", "rows", "Height of the images used", false, 480, "nb_rows", cmd);
    TCLAP::ValueArg<std::string> output_arg("o", "output", "Path to which the model is saved", false, "model.json",
                                            "output.json", cmd);
    TCLAP::SwitchArg fix_k1_arg("", "fix-k1", "Use K1 parameter for calibration", cmd);
    TCLAP::SwitchArg fix_k2_arg("", "fix-k2", "Use K2 parameter for calibration", cmd);
    TCLAP::SwitchArg fix_k3_arg("", "fix-k3", "Use K3 parameter for calibration", cmd);
    TCLAP::SwitchArg fix_tangential_arg("", "fix-tangential", "Fix tangential parameters for distortion", cmd);

    cmd.parse(argc, argv);

    MarkerSeenCollection markers_seen;
    MarkerCollection markers;

    markers.loadFile(markers_arg.getValue());
    markers_seen.loadFile(markers_seen_arg.getValue());

    cv::Size img_size(cols_arg.getValue(), rows_arg.getValue());

    int calibration_flags = CV_CALIB_FIX_ASPECT_RATIO;
    if (fix_k1_arg.getValue())
    {
      calibration_flags |= CV_CALIB_FIX_K1;
    }
    if (fix_k2_arg.getValue())
    {
      calibration_flags |= CV_CALIB_FIX_K2;
    }
    if (fix_k3_arg.getValue())
    {
      calibration_flags |= CV_CALIB_FIX_K3;
    }
    if (fix_tangential_arg.getValue())
    {
      calibration_flags |= CV_CALIB_ZERO_TANGENT_DIST;
    }
    
    CameraModel model = calibrateCamera(markers, markers_seen, nb_images_arg.getValue(), img_size, calibration_flags);
    model.saveFile(output_arg.getValue());
  }
  catch (const ArgException& exc)
  {
    std::cerr << "error: " << exc.what() << " for arg " << exc.argId() << std::endl;
    exit(EXIT_FAILURE);
  }
}
  

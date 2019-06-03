#include <rhoban_model_learning/camera_calibration/marker_collection.h>
#include <rhoban_model_learning/camera_calibration/marker_seen_collection.h>

#include <tclap/CmdLine.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace rhoban_model_learning;
using namespace TCLAP;

MarkerCollection getChessboardMarkers(const cv::Size& pattern_size, double marker_size)
{
  MarkerCollection markers;
  int marker_id = 0;
  for (int row = 0; row < pattern_size.height; row++)
  {
    for (int col = 0; col < pattern_size.width; col++)
    {
      markers[marker_id++] = Eigen::Vector3d(row * marker_size, col * marker_size, 0);
    }
  }
  return markers;
}

/**
 * Return true on success
 */
bool detectChessboard(const cv::Mat& img, const cv::Size& pattern_size, std::vector<cv::Point2f>* corners)
{
  cv::Mat gray;
  cv::cvtColor(img, gray, CV_BGR2GRAY);

  bool success = cv::findChessboardCorners(gray, pattern_size, *corners);
  if (success)
  {
    cv::TermCriteria criteria(cv::TermCriteria::Type::COUNT + cv::TermCriteria::Type::EPS, 50, 0.0001);
    cv::cornerSubPix(gray, *corners, cv::Size(5, 5), cv::Size(-1, -1), criteria);
  }
  return success;
}

MarkerSeenCollection detectChessboard(const std::vector<std::string>& img_paths, const cv::Size& pattern_size,
                                      bool display, bool ycbcr)
{
  MarkerSeenCollection markers;
  for (size_t img_id = 0; img_id < img_paths.size(); img_id++)
  {
    cv::Mat img = cv::imread(img_paths[img_id]);
    cv::Mat bgr = img;
    if (ycbcr)
    {
      cv::cvtColor(img, bgr, CV_YCrCb2BGR);
    }
    std::vector<cv::Point2f> corners;
    bool success = detectChessboard(img, pattern_size, &corners);
    if (success)
    {
      int marker_id = 0;
      for (const cv::Point2f& corner : corners)
      {
        markers[img_id][marker_id++] = Eigen::Vector2d(corner.x, corner.y);
      }
      if (display)
      {
        cv::drawChessboardCorners(bgr, pattern_size, corners, success);
      }
    }
    if (display)
    {
      cv::imshow("img", bgr);
      cv::waitKey(0);
    }
  }
  return markers;
}

MarkerSeenCollection detectChessboard(const std::string& video_path, const cv::Size& pattern_size, bool display,
                                      bool ycbcr)
{
  MarkerSeenCollection markers;
  cv::VideoCapture video;
  if (!video.open(video_path))
  {
    throw std::runtime_error("Failed to open video '" + video_path + "'");
  }
  int img_id = 0;
  while (true)
  {
    cv::Mat img;
    video >> img;
    if (img.empty())
      break;
    cv::Mat bgr = img;
    if (ycbcr)
    {
      cv::cvtColor(img, bgr, CV_YCrCb2BGR);
    }
    std::vector<cv::Point2f> corners;
    bool success = detectChessboard(img, pattern_size, &corners);
    if (success)
    {
      int marker_id = 0;
      for (const cv::Point2f& corner : corners)
      {
        markers[img_id][marker_id++] = Eigen::Vector2d(corner.x, corner.y);
      }
      if (display)
      {
        cv::drawChessboardCorners(bgr, pattern_size, corners, success);
      }
    }
    if (display)
    {
      cv::imshow("img", bgr);
      cv::waitKey(0);
    }
    img_id++;
  }
  return markers;
}

int main(int argc, char** argv)
{
  try
  {
    CmdLine cmd("Extract markers from images and writes their position in a csv file ", ' ', "0.1");

    UnlabeledMultiArg<std::string> paths_arg("input", "path to multiple images or to a video", true, "paths", cmd);
    std::vector<std::string> allowed_types = { "chessboard", "charuco" };
    ValuesConstraint<std::string> allowed_types_constraint(allowed_types);
    ValueArg<std::string> type_arg("t", "type", "The type of markers to detect: chessboard or charuco", true,
                                   "chessboard", &allowed_types_constraint, cmd);
    SwitchArg display_arg("d", "display", "Display images and wait between each image", cmd);
    SwitchArg verbose_arg("v", "verbose", "Display progress", cmd);
    SwitchArg ycrcb_arg("", "ycrcb", "Consider input as ycrcb images", cmd);
    ValueArg<int> rows_arg("r", "rows", "Number of rows in pattern", false, 9, "nbRows", cmd);
    ValueArg<int> cols_arg("c", "cols", "Number of cols in pattern", false, 6, "nbCols", cmd);
    ValueArg<double> marker_size_arg("m", "marker_size", "Size of a marker in meters", false, 0.02, "markerSize[m]",
                                     cmd);

    cmd.parse(argc, argv);

    MarkerSeenCollection markers_seen;
    MarkerCollection markers;

    cv::Size pattern_size(cols_arg.getValue(), rows_arg.getValue());

    if (type_arg.getValue() == "chessboard")
    {
      markers = getChessboardMarkers(pattern_size, marker_size_arg.getValue());
      if (paths_arg.getValue().size() == 1 && paths_arg.getValue()[0].find(".avi") != std::string::npos)
      {
        markers_seen =
            detectChessboard(paths_arg.getValue()[0], pattern_size, display_arg.getValue(), ycrcb_arg.getValue());
      }
      else
      {
        markers_seen =
            detectChessboard(paths_arg.getValue(), pattern_size, display_arg.getValue(), ycrcb_arg.getValue());
      }
    }
    else
    {
      throw std::logic_error("Charuco not implemented");
    }

    markers.saveFile("markers.csv");
    markers_seen.saveFile("markers_seen.csv");
  }
  catch (const ArgException& exc)
  {
    std::cerr << "error: " << exc.what() << " for arg " << exc.argId() << std::endl;
    exit(EXIT_FAILURE);
  }
}

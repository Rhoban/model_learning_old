#include <tclap/CmdLine.h>
#include <rhoban_utils/tables/string_table.h>
#include <rhoban_model_learning/camera_calibration/marker_seen_collection.h>
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

using namespace rhoban_model_learning;
using namespace TCLAP;

cv::Point2f getCenter(std::vector<cv::Point2f> corners)
{
  cv::Point2f center(0, 0);
  for (const cv::Point2f corner : corners)
  {
    center += corner / (float)corners.size();
  }
  return center;
}

int main(int argc, char** argv)
{
  try
  {
    CmdLine cmd("Extract aruco markers from images and writes their position in a csv file ", ' ', "0.1");

    TCLAP::ValueArg<std::string> videoPathArg("f", "video_path", "path to video", false, "video.avi", "video.avi", cmd);

    TCLAP::ValueArg<std::string> modeArg("m", "mode", "mode (write, show, find)", true, "", "mode", cmd);

    TCLAP::ValueArg<int> indexArg("i", "index", "image index", true, -1, "index", cmd);

    cmd.parse(argc, argv);

    std::string videoPath = videoPathArg.getValue();
    std::string mode = modeArg.getValue();
    int image_index_to_find = indexArg.getValue();

    cv::VideoCapture video;
    if (!video.open(videoPath))
    {
      throw std::runtime_error("failed to open video '" + videoPath + "'");
    }
    std::cout << "Definition des outils opencv." << std::endl;
    cv::Ptr<cv::aruco::DetectorParameters> params = new cv::aruco::DetectorParameters();
    std::cout << "Parametrage du detecteur." << std::endl;
    params->adaptiveThreshConstant = 7;
    std::cout << "adaptiveThreshConstant: " << params->adaptiveThreshConstant << std::endl;
    params->adaptiveThreshWinSizeMin = 1;
    std::cout << "adaptiveThreshWinSizeMin: " << params->adaptiveThreshWinSizeMin << std::endl;
    params->adaptiveThreshWinSizeMax = 23;
    std::cout << "adaptiveThreshWinSizeMax: " << params->adaptiveThreshWinSizeMax << std::endl;
    params->adaptiveThreshWinSizeStep = 2;
    std::cout << "adaptiveThreshWinSizeStep: " << params->adaptiveThreshWinSizeStep << std::endl;
    params->doCornerRefinement = true;
    std::cout << "refinement: true" << std::endl;
    params->cornerRefinementMaxIterations = 50;
    std::cout << "cornerRefinementMaxIterations: " << params->cornerRefinementMaxIterations << std::endl;
    params->cornerRefinementMinAccuracy = 0.01;
    std::cout << "cornerRefinementMinAccuracy: " << params->cornerRefinementMinAccuracy << std::endl;
    params->cornerRefinementWinSize = 1;
    std::cout << "cornerRefinementWinSize: " << params->cornerRefinementWinSize << std::endl;

    std::cout << "Creation du dictionnaire." << std::endl;
    cv::Ptr<cv::aruco::Dictionary> dic = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);

    rhoban_utils::StringTable markers_data(
        { "image_id", "marker_id", "corner_id", "pixel_x", "pixel_y" },
        { { "image_id", {} }, { "marker_id", {} }, { "corner_id", {} }, { "pixel_x", {} }, { "pixel_y", {} } });

    int image_id = 0;
    while (true)
    {
      if (!video.set(cv::CAP_PROP_POS_FRAMES, image_id))
      {
        break;
      }

      std::cout << image_id << std::endl;

      if (image_index_to_find != -1 and image_id != image_index_to_find)
      {
        image_id++;
        continue;
      }

      cv::Mat img;
      video.read(img);

      if (img.empty())
      {
        break;
      }
      cv::Mat gray;
      cv::cvtColor(img, gray, CV_BGR2GRAY);

      std::vector<int> marker_ids;
      std::vector<std::vector<cv::Point2f>> marker_corners;
      cv::aruco::detectMarkers(gray, dic, marker_corners, marker_ids, params);

      if (mode == "show")
      {
        cv::aruco::drawDetectedMarkers(gray, marker_corners, marker_ids, cv::Scalar(0, 0, 255));

        int width, height;
        int higherSize = 320;
        if (gray.cols > gray.rows)
        {
          width = higherSize;
          height = gray.rows / (gray.cols / higherSize);
        }

        else
        {
          height = higherSize;
          width = gray.cols / (gray.rows / higherSize);
        }

        cv::namedWindow("toto");  // Create a window for display.
        cv::resizeWindow("toto", width, height);
        imshow("toto", gray);  // Show our image inside it.

        cv::waitKey(0);  // Wait for a keystroke in the window
      }
      else if (mode == "write")
      {
        for (size_t i = 0; i < marker_ids.size(); i++)
        {
          for (int j = 0; j < 4; j++)
          {
            std::map<std::string, std::string> row = { { "image_id", std::to_string(image_id) },
                                                       { "marker_id", std::to_string(marker_ids[i]) },
                                                       { "corner_id", std::to_string(j) },
                                                       { "pixel_x", std::to_string(marker_corners[i][j].x) },
                                                       { "pixel_y", std::to_string(marker_corners[i][j].y) } };
            markers_data.insertRow(row);
          }
        }
      }
      else
      {
        std::cerr << "wrong mode, it should be show or write.";
        exit(EXIT_FAILURE);
      }
      image_id++;
    }

    markers_data.writeFile("markers_data.csv");
  }
  catch (const ArgException& exc)
  {
    std::cerr << "error: " << exc.what() << " for arg " << exc.argId() << std::endl;
    exit(EXIT_FAILURE);
  }
}

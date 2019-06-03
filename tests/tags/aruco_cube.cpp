#include "rhoban_model_learning/tags/aruco_cube.h"

#include <gtest/gtest.h>

using namespace std;
using namespace rhoban_model_learning;

string getAbsoluteTestFilePrefix()
{
  string filePath = __FILE__;
  string currentDirPath = filePath.substr(0, filePath.rfind("/"));
  return currentDirPath + "/../ressources/tags/";
}

TEST(ArucoCube, singleElement)
{
  ArucoCube cube;
  cube.loadFile(getAbsoluteTestFilePrefix() + "aruco_cube_1.json");
  std::map<int, ArucoTag> markers = cube.getMarkers();
  EXPECT_EQ((size_t)7, markers.size());
  std::vector<int> expected_markers = { 1, 2, 3, 4, 5, 6, 8 };
  std::vector<Eigen::Vector3d> expected_positions = {
    Eigen::Vector3d(0.65, -0.05, -0.9), Eigen::Vector3d(0.65, 0.05, -0.9),  Eigen::Vector3d(0.65, -0.05, -1.0),
    Eigen::Vector3d(0.65, 0.05, -1.0),  Eigen::Vector3d(0.65, -0.05, -1.1), Eigen::Vector3d(0.65, 0.05, -1.1),
    Eigen::Vector3d(0.35, 0.00, -1.0)
  };
  for (size_t idx = 0; idx < expected_markers.size(); idx++)
  {
    int marker_id = expected_markers[idx];
    const Eigen::Vector3d& expected_pos = expected_positions[idx];
    EXPECT_EQ((size_t)1, markers.count(marker_id));
    ArucoTag tag = markers.at(marker_id);
    std::cout << "marker_id : " << marker_id << std::endl;
    EXPECT_NEAR(expected_pos(0), tag.marker_center(0), 0.001);
    EXPECT_NEAR(expected_pos(1), tag.marker_center(1), 0.001);
    EXPECT_NEAR(expected_pos(2), tag.marker_center(2), 0.001);
  }
}
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

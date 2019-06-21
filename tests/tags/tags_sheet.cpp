#include "rhoban_model_learning/tags/tags_sheet.h"

#include <gtest/gtest.h>

using namespace std;
using namespace rhoban_model_learning;

string getAbsoluteTestFilePrefix()
{
  string filePath = __FILE__;
  string currentDirPath = filePath.substr(0, filePath.rfind("/"));
  return currentDirPath + "/../ressources/tags/";
}

TEST(TagsSheet, singleElement)
{
  TagsSheet s;
  s.loadFile(getAbsoluteTestFilePrefix() + "sheet_1x1.json");
  std::map<int, ArucoTag> markers = s.getMarkers();
  EXPECT_EQ((size_t)1, markers.size());
  EXPECT_EQ((size_t)1, markers.count(32));
  ArucoTag tag = markers.at(32);
  EXPECT_EQ(0, tag.marker_center(0));
  EXPECT_EQ(0, tag.marker_center(1));
  EXPECT_EQ(1, tag.marker_center(2));
  EXPECT_NEAR(0.707, tag.orientation.w(), 0.01);
  EXPECT_NEAR(0.0, tag.orientation.x(), 0.01);
  EXPECT_NEAR(0.707, tag.orientation.y(), 0.01);
  EXPECT_NEAR(0.0, tag.orientation.z(), 0.01);
}

TEST(TagsSheet, twoByThree)
{
  // Expected content
  std::vector<int> markers_ids = { 2, 4, 5, 8, 1, 6 };
  std::vector<Eigen::Vector3d> expected_pos = { Eigen::Vector3d(-0.1, -0.1, 1.0), Eigen::Vector3d(-0.1, 0.0, 1.0),
                                                Eigen::Vector3d(-0.1, 0.1, 1.0),  Eigen::Vector3d(0.1, -0.1, 1.0),
                                                Eigen::Vector3d(0.1, 0.0, 1.0),   Eigen::Vector3d(0.1, 0.1, 1.0) };
  // Loading + checking
  TagsSheet s;
  s.loadFile(getAbsoluteTestFilePrefix() + "sheet_2x3.json");
  std::map<int, ArucoTag> markers = s.getMarkers();
  EXPECT_EQ(markers_ids.size(), markers.size());
  for (size_t idx = 0; idx < markers_ids.size(); idx++)
  {
    int marker_id = markers_ids[idx];
    EXPECT_EQ((size_t)1, markers.count(marker_id));
    const ArucoTag& tag = markers.at(marker_id);
    EXPECT_NEAR(expected_pos[idx](0), tag.marker_center(0), std::pow(10, -6));
    EXPECT_NEAR(expected_pos[idx](1), tag.marker_center(1), std::pow(10, -6));
    EXPECT_NEAR(expected_pos[idx](2), tag.marker_center(2), std::pow(10, -6));
  }
}

TEST(TagsSheet, twoByThreeRPY)
{
  // Expected content
  std::vector<int> markers_ids = { 2, 4, 5, 8, 1, 6 };
  std::vector<Eigen::Vector3d> expected_pos = { Eigen::Vector3d(-0.1, -0.1, 1.0), Eigen::Vector3d(-0.1, 0.0, 1.0),
                                                Eigen::Vector3d(-0.1, 0.1, 1.0),  Eigen::Vector3d(0.1, -0.1, 1.0),
                                                Eigen::Vector3d(0.1, 0.0, 1.0),   Eigen::Vector3d(0.1, 0.1, 1.0) };
  // Loading + checking
  TagsSheet s;
  s.loadFile(getAbsoluteTestFilePrefix() + "sheet_2x3_rpy.json");
  std::map<int, ArucoTag> markers = s.getMarkers();
  EXPECT_EQ(markers_ids.size(), markers.size());
  for (size_t idx = 0; idx < markers_ids.size(); idx++)
  {
    int marker_id = markers_ids[idx];
    EXPECT_EQ((size_t)1, markers.count(marker_id));
    const ArucoTag& tag = markers.at(marker_id);
    EXPECT_NEAR(expected_pos[idx](0), tag.marker_center(0), std::pow(10, -6));
    EXPECT_NEAR(expected_pos[idx](1), tag.marker_center(1), std::pow(10, -6));
    EXPECT_NEAR(expected_pos[idx](2), tag.marker_center(2), std::pow(10, -6));
  }
}

TEST(TagsSheet, threeByTwo)
{
  // Expected content
  std::vector<int> markers_ids = { 2, 4, 5, 8, 1, 6 };
  std::vector<Eigen::Vector3d> expected_pos = { Eigen::Vector3d(-0.1, -0.1, 1.0), Eigen::Vector3d(-0.1, 0.1, 1.0),
                                                Eigen::Vector3d(0.0, -0.1, 1.0),  Eigen::Vector3d(0.0, 0.1, 1.0),
                                                Eigen::Vector3d(0.1, -0.1, 1.0),  Eigen::Vector3d(0.1, 0.1, 1.0) };
  // Loading + checking
  TagsSheet s;
  s.loadFile(getAbsoluteTestFilePrefix() + "sheet_3x2.json");
  std::map<int, ArucoTag> markers = s.getMarkers();
  EXPECT_EQ(markers_ids.size(), markers.size());
  for (size_t idx = 0; idx < markers_ids.size(); idx++)
  {
    int marker_id = markers_ids[idx];
    EXPECT_EQ((size_t)1, markers.count(marker_id));
    const ArucoTag& tag = markers.at(marker_id);
    EXPECT_NEAR(expected_pos[idx](0), tag.marker_center(0), std::pow(10, -6));
    EXPECT_NEAR(expected_pos[idx](1), tag.marker_center(1), std::pow(10, -6));
    EXPECT_NEAR(expected_pos[idx](2), tag.marker_center(2), std::pow(10, -6));
  }
}
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

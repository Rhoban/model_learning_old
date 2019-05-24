#include "rhoban_model_learning/humanoid_models/infered_poses_data_set_reader.h"

#include <gtest/gtest.h>

using namespace std;
using namespace rhoban_model_learning;

typedef InferedPosesHumanoidDataSetReader IPHDSR;

string getAbsoluteTestFilePrefix()
{
  string filePath = __FILE__;
  string currentDirPath = filePath.substr(0, filePath.rfind("/"));
  return currentDirPath + "/../ressources/humanoid_models/";
}

TEST(InferedPosesHumanoidDataSetReader, artificialFile)
{
  IPHDSR reader;
  std::default_random_engine engine;
  reader.loadFile(getAbsoluteTestFilePrefix() + "infered_poses_data_set_reader.json");
  DataSet data = reader.extractSamples(getAbsoluteTestFilePrefix() + "infered_poses_data.csv", &engine);
  EXPECT_EQ((size_t)8, data.training_set.size());
  EXPECT_EQ((size_t)4, data.validation_set.size());
}
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

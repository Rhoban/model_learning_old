#include <gtest/gtest.h>

#include "rhoban_model_learning/composite_model.h"

using namespace rhoban_model_learning;

class ModelA : public Model
{
public:
  ModelA() : a(0.0), b(1.0)
  {
  }

  Eigen::VectorXd getParameters() const override
  {
    return Eigen::Vector2d(a, b);
  }
  void setParameters(const Eigen::VectorXd& new_parameters) override
  {
    a = new_parameters(0);
    b = new_parameters(1);
  }

  std::vector<std::string> getParametersNames() const override
  {
    return { "a", "b" };
  }

  void fromJson(const Json::Value& json_value, const std::string& dir_name) override
  {
    (void)dir_name;
    rhoban_utils::tryRead(json_value, "a", &a);
    rhoban_utils::tryRead(json_value, "b", &b);
  }

  Json::Value toJson() const override
  {
    Json::Value v;
    v["a"] = a;
    v["b"] = b;
    return v;
  }

  std::string getClassName() const
  {
    return "ModelA";
  }

private:
  double a;
  double b;
};

class ModelB : public Model
{
public:
  ModelB() : a(0), b(1), c(-1)
  {
  }

  Eigen::VectorXd getParameters() const override
  {
    return Eigen::Vector3d(a, b, c);
  }
  void setParameters(const Eigen::VectorXd& new_parameters) override
  {
    a = (int)new_parameters(0);
    b = (int)new_parameters(1);
    c = (int)new_parameters(2);
  }

  std::vector<std::string> getParametersNames() const override
  {
    return { "a", "b", "c" };
  }

  void fromJson(const Json::Value& json_value, const std::string& dir_name) override
  {
    (void)dir_name;
    rhoban_utils::tryRead(json_value, "a", &a);
    rhoban_utils::tryRead(json_value, "b", &b);
    rhoban_utils::tryRead(json_value, "c", &c);
  }

  Json::Value toJson() const override
  {
    Json::Value v;
    v["a"] = a;
    v["b"] = b;
    v["c"] = c;
    return v;
  }

  std::string getClassName() const
  {
    return "ModelB";
  }

private:
  int a, b, c;
};

class AggregateModel : public CompositeModel
{
public:
  AggregateModel()
  {
    models["B"] = std::unique_ptr<Model>(new ModelB());
    models["A1"] = std::unique_ptr<Model>(new ModelA());
    models["A2"] = std::unique_ptr<Model>(new ModelA());
  }

  std::string getClassName() const
  {
    return "AggregateModel";
  }

  std::unique_ptr<Model> clone() const
  {
    return std::unique_ptr<Model>(new AggregateModel(*this));
  }
};

TEST(CompositeModel, basicBuilder)
{
  AggregateModel model;
  EXPECT_EQ(7, model.getParametersSize());  // Empty model has only bias and std_dev
  std::vector<std::string> expected_names = { "A1:a", "A1:b", "A2:a", "A2:b", "B:a", "B:b", "B:c" };
  std::vector<std::string> received_names = model.getParametersNames();
  ASSERT_EQ(expected_names.size(), received_names.size());
  for (size_t idx = 0; idx < received_names.size(); idx++)
  {
    EXPECT_EQ(expected_names[idx], received_names[idx]);
  }
}

TEST(CompositeModel, getIndicesFromName)
{
  AggregateModel model;
  // Test indices
  std::vector<int> expected_indices = { 2, 3, 5 };
  std::set<int> received_indices = model.getIndicesFromNames({ "A2", "B:b" });
  ASSERT_EQ(expected_indices.size(), received_indices.size());
  for (int idx : expected_indices)
  {
    EXPECT_EQ(1, (int)received_indices.count(idx));
  }
  // Test of all
  received_indices = model.getIndicesFromNames({ "all" });
  ASSERT_EQ(7, (int)received_indices.size());
  for (int idx = 0; idx < 7; idx++)
  {
    EXPECT_EQ(1, (int)received_indices.count(idx));
  }
  // Test of duplicated index
  EXPECT_THROW(model.getIndicesFromNames({ "A1:a", "all" }), std::runtime_error);
  // Test of invalid name
  EXPECT_THROW(model.getIndicesFromNames({ "A1:toto" }), std::out_of_range);
}

TEST(CompositeModel, splitIndiceesAmongSubModels)
{
  AggregateModel model;
  // Test indices
  std::map<std::string, std::set<int>> expected_indices;
  expected_indices["A1"] = { 0 };
  expected_indices["A2"] = { 0, 1 };
  expected_indices["B"] = { 0, 2 };

  std::set<int> indices = model.getIndicesFromNames({ "A1:a", "A2:all", "B:a", "B:c" });
  std::map<std::string, std::set<int>> splitted_indices = model.splitIndicesAmongSubModels(indices);
  for (auto& entry : splitted_indices)
  {
    EXPECT_EQ(expected_indices[entry.first], splitted_indices[entry.first]);
    for (size_t i = 0; i < expected_indices[entry.first].size(); i++)
    {
      EXPECT_TRUE(expected_indices[entry.first] == splitted_indices[entry.first]);
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

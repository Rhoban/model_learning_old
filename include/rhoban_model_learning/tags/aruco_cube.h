#pragma once

#include "rhoban_model_learning/model.h"
#include "rhoban_model_learning/tags/tags_sheet.h"
#include "rhoban_model_learning/tags/tags_collection.h"
#include "rhoban_model_learning/basic_models/pose_model.h"

namespace rhoban_model_learning
{
/// Represent a cube which can contain an aruco tag on every side face (i.e. not
/// top neither bottom)
///
/// Faces centers are defined as following:
/// - 0: cubeCenter + ( halfSide,        0, 0)
/// - 1: cubeCenter + (        0, halfSide, 0)
/// - 2: cubeCenter + (-halfSide,        0, 0)
/// - 3: cubeCenter + (        0,-halfSide, 0)
class ArucoCube : public Model, public TagsCollection
{
public:
  ArucoCube();
  ArucoCube(const ArucoCube& other);

  std::map<int, ArucoTag> getMarkers() const override;

  int getParametersSize() const override;

  Eigen::VectorXd getParameters() const override;
  void setParameters(const Eigen::VectorXd& new_params) override;
  std::vector<std::string> getParametersNames() const override;

  virtual Json::Value toJson() const override;
  virtual void fromJson(const Json::Value& v, const std::string& dir_path) override;
  virtual std::string getClassName() const override;

private:
  /// Update the pose of the sheets to match the pose of the cube
  void updateSheets();

  /// The pose of the cube
  PoseModel pose;

  /// The distance between two opposite sides
  double side;

  /// The sheets allowed inside the cube
  std::map<int, TagsSheet> sheets;
};

}  // namespace rhoban_model_learning

#include "rhoban_model_learning/basic_models/multi_poses_model.h"

#include <rhoban_utils/util.h>

namespace rhoban_model_learning
{
MultiPosesModel::MultiPosesModel() : Model()
{
}

MultiPosesModel::MultiPosesModel(const MultiPosesModel& other) : Model(other), poses(other.poses)
{
}

const PoseModel& MultiPosesModel::getPose(int index) const
{
  return poses[index];
}

int MultiPosesModel::getParametersSize() const
{
  if (poses.size() == 0)
    return 0;
  return poses[0].getParametersSize() * poses.size();
}

Eigen::VectorXd MultiPosesModel::getParameters() const
{
  Eigen::VectorXd parameters(getParametersSize());
  int param_idx = 0;
  for (const PoseModel& pose : poses)
  {
    int pose_params = pose.getParametersSize();
    parameters.segment(param_idx, pose_params) = pose.getParameters();
    param_idx += pose_params;
  }
  return parameters;
}

void MultiPosesModel::setParameters(const Eigen::VectorXd& new_params)
{
  if (new_params.rows() != getParametersSize())
  {
    throw std::runtime_error(DEBUG_INFO + " invalid size for new_params, expecting " +
                             std::to_string(getParametersSize()) + ", got " + std::to_string(new_params.rows()));
  }
  int param_idx = 0;
  for (PoseModel& pose : poses)
  {
    int pose_params = pose.getParametersSize();
    pose.setParameters(new_params.segment(param_idx, pose_params));
    param_idx += pose_params;
  }
}

std::vector<std::string> MultiPosesModel::getParametersNames() const
{
  std::vector<std::string> names;
  int pose_idx = 0;
  for (const PoseModel& pose : poses)
  {
    std::string prefix = "pose" + std::to_string(pose_idx) + ":";
    for (const std::string& inner_name : pose.getParametersNames())
    {
      names.push_back(prefix + inner_name);
    }
    pose_idx++;
  }
  return names;
}

Json::Value MultiPosesModel::toJson() const
{
  Json::Value v;
  for (Json::ArrayIndex idx = 0; idx < poses.size(); idx++)
  {
    v[idx] = poses[idx].toJson();
  }
  return v;
}

void MultiPosesModel::fromJson(const Json::Value& v, const std::string& dir_name)
{
  if (v.isArray())
  {
    poses.resize(v.size());
    for (Json::ArrayIndex idx = 0; idx < v.size(); idx++)
    {
      poses[idx].fromJson(v[idx], dir_name);
    }
  }
  else if (v.isObject() && v.isMember("nb_poses"))
  {
    int nb_poses = rhoban_utils::read<int>(v, "nb_poses");
    poses.clear();
    poses.resize(nb_poses);
  }
  else
  {
    throw rhoban_utils::JsonParsingError(DEBUG_INFO + " v is not an array and has no attribut nb_poses");
  }
}

std::string MultiPosesModel::getClassName() const
{
  return "MultiPosesModel";
}

std::unique_ptr<Model> MultiPosesModel::clone() const
{
  return std::unique_ptr<Model>(new MultiPosesModel(*this));
}

}  // namespace rhoban_model_learning

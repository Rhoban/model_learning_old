#include "rhoban_model_learning/basic_models/doubles_model.h"

#include <rhoban_utils/util.h>

namespace rhoban_model_learning
{
DoublesModel::DoublesModel()
{
}

DoublesModel::DoublesModel(const DoublesModel& other) : Model(other), params(other.params)
{
}

int DoublesModel::getParametersSize() const
{
  return params.size();
}

Eigen::VectorXd DoublesModel::getParameters() const
{
  return Eigen::VectorXd(params);
}

void DoublesModel::setParameters(const Eigen::VectorXd& new_params)
{
  if (new_params.size() != params.size())
  {
    throw std::runtime_error(DEBUG_INFO + "New parameters are of size " + std::to_string(new_params.size()) +
                             " instead of " + std::to_string(params.size()));
  }
  params = new_params;
}

std::vector<std::string> DoublesModel::getParametersNames() const
{
  std::vector<std::string> params_names;
  for (int i = 0; i < params.size(); i++)
  {
    params_names.push_back("param" + std::to_string(i));
  }
  return params_names;
}

double DoublesModel::getParameter(int i) const
{
  return params(i);
}

Json::Value DoublesModel::toJson() const
{
  Json::Value v;
  v["params"] = rhoban_utils::vector2Json(params);
  return v;
}

void DoublesModel::fromJson(const Json::Value& v, const std::string& dir_name)
{
  (void)dir_name;
  rhoban_utils::tryReadEigen(v, "params", &params);
}

std::string DoublesModel::getClassName() const
{
  return "DoublesModel";
}

std::unique_ptr<Model> DoublesModel::clone() const
{
  return std::unique_ptr<Model>(new DoublesModel(*this));
}

}  // namespace rhoban_model_learning

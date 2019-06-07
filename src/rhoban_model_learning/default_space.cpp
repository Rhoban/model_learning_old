#include "rhoban_model_learning/default_space.h"

namespace rhoban_model_learning
{
DefaultSpace::DefaultSpace()
{
}

DefaultSpace::DefaultSpace(const DefaultSpace& other) : space(other.space)
{
}

Eigen::MatrixXd DefaultSpace::getParametersSpace(const Model& m, const ModelPrior& p) const
{
  (void)m;
  (void)p;
  return space;
}

std::string DefaultSpace::getClassName() const
{
  return "DefaultSpace";
}

void DefaultSpace::fromJson(const Json::Value& json_value, const std::string& dir_name)
{
  (void)dir_name;
  space = rhoban_utils::readEigen<-1, -1>(json_value, "space");
}

Json::Value DefaultSpace::toJson() const
{
  Json::Value v;
  v["space"] = rhoban_utils::matrix2Json(space);
  return v;
}

std::unique_ptr<ModelSpace> DefaultSpace::clone() const
{
  return std::unique_ptr<ModelSpace>(new DefaultSpace(*this));
}

}  // namespace rhoban_model_learning

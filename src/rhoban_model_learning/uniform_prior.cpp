#include "rhoban_model_learning/uniform_prior.h"

namespace rhoban_model_learning
{
UniformPrior::UniformPrior()
{
}

std::string UniformPrior::getClassName() const
{
  return "UniformPrior";
}

void UniformPrior::fromJson(const Json::Value& json_value, const std::string& dir_name)
{
  (void)dir_name;
  mins = rhoban_utils::readEigen<-1, 1>(json_value, "mins");
  maxs = rhoban_utils::readEigen<-1, 1>(json_value, "maxs");
}

Json::Value UniformPrior::toJson() const
{
  Json::Value v;
  v["mins"] = rhoban_utils::vector2Json(mins);
  v["maxs"] = rhoban_utils::vector2Json(maxs);
  return v;
}

}  // namespace rhoban_model_learning

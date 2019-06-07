#include "rhoban_model_learning/model_space.h"
#include "rhoban_model_learning/model_space_factory.h"

#include "rhoban_model_learning/model.h"

namespace rhoban_model_learning
{
Eigen::MatrixXd ModelSpace::getParametersSpace(const Model& m, const ModelPrior& prior,
                                               const std::set<int>& used_indices) const
{
  Eigen::MatrixXd global_space = getParametersSpace(m, prior);
  Eigen::MatrixXd used_space(used_indices.size(), 2);
  int used_idx = 0;
  for (int idx : used_indices)
  {
    used_space.row(used_idx) = global_space.row(idx);
    used_idx++;
  }
  return used_space;
}

void ModelSpace::append(const Model& m, const ModelPrior& prior, const std::set<int>& used_indices,
                        std::ostream& out) const
{
  std::vector<std::string> parameters_names = m.getParametersNames(used_indices);
  Eigen::MatrixXd parameters_spaces = getParametersSpace(m, prior, used_indices);
  for (int i = 0; i < parameters_spaces.rows(); i++)
  {
    out << parameters_names[i] << ": [" << parameters_spaces(i, 0) << "," << parameters_spaces(i, 1) << "]"
        << std::endl;
  }
}

std::unique_ptr<ModelSpace> ModelSpace::clone() const
{
  Json::Value v = toJson();
  std::unique_ptr<ModelSpace> other = ModelSpaceFactory().build(getClassName());
  other->fromJson(v, "./");
  return other;
}

}  // namespace rhoban_model_learning

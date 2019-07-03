#pragma once

#include "rhoban_model_learning/model.h"

#include <rhoban_utils/util.h>

namespace rhoban_model_learning
{
/// Implements composition of models with respect to parameters handling and
/// serialization
///
/// A few important points:
/// - Constructors of the child class has to register the models in the map
///   - Except copy constructors (just call copy constructors of CompositeModel)
/// - Only 'mandatory' function to implement for the child class are:
///   'getClassName' and 'clone'
class CompositeModel : public Model
{
public:
  CompositeModel();
  CompositeModel(const CompositeModel& other);

  /// Return the first index for parameters at given name
  int getOffset(const std::string& name) const;

  /// Throws std::out_of_range on invalid name
  const Model& getModel(const std::string& name) const;

  int getParametersSize() const override;

  Eigen::VectorXd getParameters() const override;
  void setParameters(const Eigen::VectorXd& new_params) override;
  std::vector<std::string> getParametersNames() const override;

  std::set<int> getIndicesFromName(const std::string& name) const override;

  std::map<std::string, std::set<int>> splitIndicesAmongSubModels(const std::set<int>& indices) const;

  Json::Value toJson() const override;
  void fromJson(const Json::Value& v, const std::string& dir_name) override;

  // Throws explicit runtime error if models at given key is not of class T
  template <class T>
  void checkType(const std::string& key) const
  {
    try
    {
      dynamic_cast<const T&>(*models.at(key));
    }
    catch (const std::bad_cast& e)
    {
      throw std::runtime_error(DEBUG_INFO + " invalid type for '" + key + "'");
    }
  }

protected:
  // Each model is named to facilitate use of prefixes
  std::map<std::string, std::unique_ptr<Model>> models;
};

}  // namespace rhoban_model_learning

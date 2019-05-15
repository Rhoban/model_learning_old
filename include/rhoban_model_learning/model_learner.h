#pragma once

#include "rhoban_model_learning/model.h"
#include "rhoban_model_learning/model_prior.h"
#include "rhoban_model_learning/model_space.h"
#include "rhoban_model_learning/predictor.h"
#include "rhoban_model_learning/sample.h"

#include "rhoban_bbo/optimizer.h"

namespace rhoban_model_learning
{
/// A class allowing to optimize the parameters of a model using logLikelihood
class ModelLearner : public rhoban_utils::JsonSerializable
{
public:
  struct Result
  {
    std::unique_ptr<Model> model;
    double training_log_likelihood;
    double validation_log_likelihood;
  };

  ModelLearner();

  ModelLearner(std::unique_ptr<Model> model, std::unique_ptr<ModelPrior> prior, std::unique_ptr<ModelSpace> space,
               std::unique_ptr<Predictor> predictor, std::unique_ptr<rhoban_bbo::Optimizer> optimizer,
               const std::set<int> trainable_indices);

  Result learnParameters(const DataSet& data_set, std::default_random_engine* engine);

  Result learnParameters(const SampleVector& training_set, const SampleVector& validation_set,
                         std::default_random_engine* engine);

  /// Return the loglikelihood of the provided model according to both, prior and data
  double getLogLikelihood(const Model& model, const SampleVector& data_set, std::default_random_engine* engine) const;

  virtual std::string getClassName() const override;
  Json::Value toJson() const override;

  /// Training indices can be provided either using parameters indices (trainable_indices)
  /// or using parameter names (indices_names)
  void fromJson(const Json::Value& v, const std::string& dir_name) override;

  const Model& getModel() const;

  const ModelPrior& getPrior() const;

  const ModelSpace& getSpace() const;

  const std::set<int>& getTrainableIndices() const;

  void exportValidationResulstToCSV(const Model& _model, const DataSet& data, const std::string& filename,
                                    char separator = ',') const;

protected:
  /// The model which will be learnt
  std::unique_ptr<Model> model;

  /// The prior used for learning
  std::unique_ptr<ModelPrior> prior;

  /// The allowed space
  std::unique_ptr<ModelSpace> space;

  /// The predictor used to compare model prediction and observations
  std::unique_ptr<Predictor> predictor;

  /// The blackbox optimizer used for parameters calibration
  std::unique_ptr<rhoban_bbo::Optimizer> optimizer;

  /// The subset of indices allowed for modification
  std::set<int> trainable_indices;
};

}  // namespace rhoban_model_learning

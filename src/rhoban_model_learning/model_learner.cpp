#include "rhoban_model_learning/model_learner.h"

#include "rhoban_model_learning/model_factory.h"
#include "rhoban_model_learning/model_prior_factory.h"
#include "rhoban_model_learning/model_space_factory.h"
#include "rhoban_model_learning/predictor_factory.h"

#include "rhoban_bbo/optimizer_factory.h"

using namespace rhoban_bbo;

namespace rhoban_model_learning
{
ModelLearner::ModelLearner()
{
}

ModelLearner::ModelLearner(std::unique_ptr<Model> model_, std::unique_ptr<ModelPrior> prior_,
                           std::unique_ptr<ModelSpace> space_, std::unique_ptr<Predictor> predictor_,
                           std::unique_ptr<rhoban_bbo::Optimizer> optimizer_, const std::set<int> trainable_indices_)
  : model(std::move(model_))
  , prior(std::move(prior_))
  , space(std::move(space_))
  , predictor(std::move(predictor_))
  , optimizer(std::move(optimizer_))
  , trainable_indices(trainable_indices_)
{
  prior->setInitialMean(*model);
}

ModelLearner::Result ModelLearner::learnParameters(const DataSet& data, std::default_random_engine* engine)
{
  return learnParameters(data.training_set, data.validation_set, engine);
}

ModelLearner::Result ModelLearner::learnParameters(const SampleVector& training_set, const SampleVector& validation_set,
                                                   std::default_random_engine* engine)
{
  if (training_set.size() == 0)
  {
    throw std::logic_error(DEBUG_INFO + " empty training set");
  }
  if (validation_set.size() == 0)
  {
    throw std::logic_error(DEBUG_INFO + " empty validation set");
  }

  Result result;
  rhoban_bbo::Optimizer::RewardFunc reward_function = [this, &training_set](const Eigen::VectorXd& parameters,
                                                                            std::default_random_engine* engine) {
    // Copy the original model, update the parameters and compute logLikelihood
    std::unique_ptr<Model> model_copy = this->model->clone();
    model_copy->setParameters(parameters, this->getTrainableIndices());
    return this->getLogLikelihood(*model_copy, training_set, engine);
  };
  Eigen::MatrixXd matrix_space = space->getParametersSpace(*model, *prior, trainable_indices);
  if (matrix_space.rows() == 0)
  {
    throw std::logic_error("ModelLearner::learnParameters: model has no parameters");
  }
  optimizer->setLimits(matrix_space);
  Eigen::VectorXd initial_guess = model->getParameters(trainable_indices);
  // XXX debug
  // std::cout << "Noising initial guess" << std::endl;
  // Eigen::VectorXd noised_initial_guess =
  //     prior->addNoiseToParameters(*model, initial_guess, getTrainableIndices(), engine);
  std::cout << "Noising off" << std::endl;
  Eigen::VectorXd noised_initial_guess = initial_guess;

  Eigen::VectorXd best_parameters;

  std::cout << "Start training." << std::endl;
  best_parameters = optimizer->train(reward_function, noised_initial_guess, engine);
  // Copy the model
  std::cout << "Finished training." << std::endl;
  result.model = model->clone();
  result.model->setParameters(best_parameters, getTrainableIndices());
  // Estimate log likelihood on both, training set and validation set
  result.training_log_likelihood = getLogLikelihood(*result.model, training_set, engine);
  result.validation_log_likelihood = getLogLikelihood(*result.model, validation_set, engine);
  return result;
}

double ModelLearner::getLogLikelihood(const Model& model, const SampleVector& data_set,
                                      std::default_random_engine* engine) const
{
  double data_all = predictor->averageLogLikelihood(data_set, model, engine);
  double parameters_ll = prior->getLogLikelihood(model, trainable_indices);
  // Since we use average loglikelihood for data, parameters_ll has to be
  // normalized by number of elements in data set
  return data_all + parameters_ll / data_set.size();
}

std::string ModelLearner::getClassName() const
{
  return "ModelLearner";
}

Json::Value ModelLearner::toJson() const
{
  Json::Value v;
  if (model)
  {
    v["model"] = model->toFactoryJson();
  }
  if (prior)
  {
    v["prior"] = prior->toFactoryJson();
  }
  if (space)
  {
    v["space"] = space->toFactoryJson();
  }
  if (predictor)
  {
    v["predictor"] = predictor->toFactoryJson();
  }
  if (optimizer)
  {
    v["optimizer"] = optimizer->toFactoryJson();
  }
  v["trainable_indices"] = rhoban_utils::set2Json(trainable_indices);
  return v;
}

void ModelLearner::fromJson(const Json::Value& v, const std::string& dir_name)
{
  model = ModelFactory().read(v, "model", dir_name);
  prior = ModelPriorFactory().read(v, "prior", dir_name);
  space = ModelSpaceFactory().read(v, "space", dir_name);
  predictor = PredictorFactory().read(v, "predictor", dir_name);
  optimizer = OptimizerFactory().read(v, "optimizer", dir_name);
  trainable_indices.clear();
  rhoban_utils::tryReadSet<int>(v, "trainable_indices", &trainable_indices);
  std::vector<std::string> indices_names;
  rhoban_utils::tryReadVector<std::string>(v, "indices_names", &indices_names);
  if (indices_names.size() > 0)
  {
    if (trainable_indices.size() > 0)
    {
      throw std::runtime_error(DEBUG_INFO + " both trainable_indices and indices_names specified");
    }
    trainable_indices = model->getIndicesFromNames(indices_names);
  }

  int model_size = model->getParametersSize();
  int prior_size = prior->getParametersSize();
  int space_size = space->getParametersSpace(*model, *prior).rows();
  if (model_size != prior_size)
  {
    throw std::runtime_error(DEBUG_INFO + "size of prior (" + std::to_string(prior_size) + ") size of model (" +
                             std::to_string(model_size) + ")");
  }
  if (model_size != space_size)
  {
    throw std::runtime_error(DEBUG_INFO + "size of space (" + std::to_string(space_size) + ") size of model (" +
                             std::to_string(model_size) + ")");
  }

  prior->setInitialMean(*model);
}

const Model& ModelLearner::getModel() const
{
  return *model;
}

const ModelPrior& ModelLearner::getPrior() const
{
  return *prior;
}

const ModelSpace& ModelLearner::getSpace() const
{
  return *space;
}

const std::set<int>& ModelLearner::getTrainableIndices() const
{
  return trainable_indices;
}

void ModelLearner::exportValidationResulstToCSV(const Model& _model, const DataSet& data, const std::string& filename,
                                                char separator) const
{
  predictor->exportPredictionsToCSV(_model, data.validation_set, filename, separator);
}

}  // namespace rhoban_model_learning

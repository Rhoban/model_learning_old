#include "rhoban_model_learning/model_learner.h"
#include "rhoban_model_learning/model_factory.h"
#include "rhoban_model_learning/data_set_reader_factory.h"
#include "rhoban_model_learning/model_prior_factory.h"
#include "rhoban_model_learning/predictor_factory.h"
#include "rhoban_model_learning/model_space_factory.h"

#include "rhoban_bbo/optimizer_factory.h"

#include "rhoban_utils/timing/time_stamp.h"

#include "rhoban_random/tools.h"

using namespace rhoban_bbo;
using namespace rhoban_model_learning;
using namespace rhoban_utils;

class Config : public rhoban_utils::JsonSerializable
{
public:
  Config() : nb_runs(1)
  {
  }

  std::string getClassName() const override
  {
    return "ModelLearningAnalyzerConfig";
  }

  Json::Value toJson() const override
  {
    throw std::logic_error("Not implemented");
  }

  void fromJson(const Json::Value& v, const std::string& dir_name) override
  {
    rhoban_utils::tryRead(v, "nb_runs", &nb_runs);
    readers = DataSetReaderFactory().readMap(v, "readers", dir_name);
    models = ModelFactory().readMap(v, "models", dir_name);
    optimizers = OptimizerFactory().readMap(v, "optimizers", dir_name);
    predictor = PredictorFactory().read(v, "predictor", dir_name);
    prior = ModelPriorFactory().read(v, "prior", dir_name);
    space = ModelSpaceFactory().read(v, "space", dir_name);
    rhoban_utils::tryReadVector<std::string>(v, "indices_names", &indices_names);
  }

  /// The number of runs for each configuration
  int nb_runs;

  /// All available readers
  std::map<std::string, std::unique_ptr<DataSetReader>> readers;

  /// All available models
  std::map<std::string, std::unique_ptr<Model>> models;

  /// All available optimizers
  std::map<std::string, std::unique_ptr<Optimizer>> optimizers;

  /// A predicor
  std::unique_ptr<Predictor> predictor;

  /// A prior
  std::unique_ptr<ModelPrior> prior;

  /// A space
  std::unique_ptr<ModelSpace> space;

  /// Indices names
  std::vector<std::string> indices_names;
};

int main(int argc, char** argv)
{
  if (argc < 3)
  {
    std::cerr << "Usage: " << argv[0] << " <config.json> <data_file>" << std::endl;
    exit(EXIT_FAILURE);
  }

  Config conf;
  conf.loadFile(argv[1]);

  std::string data_path(argv[2]);

  std::default_random_engine engine = rhoban_random::getRandomEngine();

  std::ofstream results_file("results.csv");

  // OPTIONAL: eventually, reduce number of columns if there is only 1 optimizer
  // or only 1 reader etc...
  results_file << "optimizer,model,reader,trainingScore,validationScore,learningTime" << std::endl;

  // For each model
  for (const auto& model_pair : conf.models)
  {
    std::string model_name = model_pair.first;
    const Model& model = *(model_pair.second);
    // Parameters file
    std::ostringstream name_oss;
    name_oss << model_name << "_parameters.csv";
    std::ofstream params_file(name_oss.str());
    std::vector<std::string> parameter_names = model.getParametersNames();
    params_file << "optimizer,reader";
    for (size_t idx = 0; idx < parameter_names.size(); idx++)
    {
      params_file << "," << parameter_names[idx];
    };
    params_file << std::endl;
    // For each optimizer
    for (const auto& optimizer_pair : conf.optimizers)
    {
      std::string optimizer_name = optimizer_pair.first;
      const Optimizer& optimizer = *(optimizer_pair.second);

      // Initialize the learning_model
      std::cout << "Cloning" << std::endl;
      ModelLearner learner(model.clone(), conf.prior->clone(), conf.space->clone(), conf.predictor->clone(),
                           optimizer.clone(), model.getIndicesFromNames(conf.indices_names));
      // For each reader
      for (const auto& reader_pair : conf.readers)
      {
        std::string reader_name = reader_pair.first;
        const DataSetReader& reader = *(reader_pair.second);
        // Perform multiple runs
        Eigen::VectorXd params_sum = Eigen::VectorXd::Zero(parameter_names.size());
        for (int run_id = 0; run_id < conf.nb_runs; run_id++)
        {
          // Extract data (splits between training and validation
          DataSet data = reader.extractSamples(data_path, &engine);
          // Learn model
          TimeStamp start = TimeStamp::now();
          ModelLearner::Result r = learner.learnParameters(data, &engine);
          TimeStamp end = TimeStamp::now();
          double learning_time = diffSec(start, end);
          // Writing scores
          results_file << optimizer_name << "," << model_name << "," << reader_name << "," << r.training_log_likelihood
                       << "," << r.validation_log_likelihood << "," << learning_time << std::endl;
          // Writing params
          Eigen::VectorXd params = r.model->getParameters();
          params_file << optimizer_name << "," << reader_name;
          for (int i = 0; i < params.rows(); i++)
          {
            params_file << "," << params(i);
          }
          params_sum += params;
          params_file << std::endl;

          // Saving params
          r.model->saveFile(model_name + "_" + optimizer_name + "_" + reader_name + "_" + std::to_string(run_id) +
                                "_trained_model.json",
                            true);
          learner.exportValidationResulstToCSV(*r.model, data,
                                               "predictionResultsTrained_" + model_name + "_" + optimizer_name + "_" +
                                                   reader_name + "_" + std::to_string(run_id) + ".csv",
                                               ',');
        }
        Eigen::VectorXd params_average = params_sum / conf.nb_runs;
        std::unique_ptr<Model> average_model = model.clone();
        average_model->setParameters(params_average);
        average_model->saveFile(model_name + "_" + optimizer_name + "_" + reader_name + "_average_trained_models.json",
                                true);
      }
    }
  }
}

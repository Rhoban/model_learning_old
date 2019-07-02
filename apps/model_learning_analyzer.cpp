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
  Config() : nb_runs_by_iteration(1), nb_iterations(1)
  {
  }

  Config(Config& other)
    : nb_runs_by_iteration(other.nb_runs_by_iteration)
    , nb_iterations(other.nb_iterations)
    , indices_names(other.indices_names)
  {
    for (const auto& entry : other.models)
    {
      models[entry.first] = entry.second->clone();
    }
    for (const auto& entry : other.readers)
    {
      readers[entry.first] = entry.second->clone();
    }
    for (const auto& entry : other.optimizers)
    {
      optimizers[entry.first] = entry.second->clone();
    }
    predictor = other.predictor->clone();
    prior = other.prior->clone();
    space = other.space->clone();
  }

  std::string getClassName() const override
  {
    return "ModelLearningAnalyzerConfig";
  }

  Json::Value toJson() const override
  {
    Json::Value v;
    for (const auto& entry : models)
    {
      v["models"][entry.first] = entry.second->toFactoryJson();
    }
    for (const auto& entry : readers)
    {
      v["readers"][entry.first] = entry.second->toFactoryJson();
    }
    for (const auto& entry : optimizers)
    {
      v["optimizers"][entry.first] = entry.second->toFactoryJson();
    }
    v["space"] = space->toFactoryJson();
    v["predictor"] = predictor->toFactoryJson();
    v["prior"] = prior->toFactoryJson();
    v["indices_names"] = vector2Json(indices_names);
    v["nb_runs_by_iteration"] = val2Json(nb_runs_by_iteration);
    v["nb_iterations"] = val2Json(nb_iterations);

    return v;
  }

  void fromJson(const Json::Value& v, const std::string& dir_name) override
  {
    rhoban_utils::tryRead(v, "nb_runs_by_iteration", &nb_runs_by_iteration);
    rhoban_utils::tryRead(v, "nb_iterations", &nb_iterations);
    readers = DataSetReaderFactory().readMap(v, "readers", dir_name);
    models = ModelFactory().readMap(v, "models", dir_name);
    optimizers = OptimizerFactory().readMap(v, "optimizers", dir_name);
    predictor = PredictorFactory().read(v, "predictor", dir_name);
    prior = ModelPriorFactory().read(v, "prior", dir_name);
    space = ModelSpaceFactory().read(v, "space", dir_name);
    rhoban_utils::tryReadVector<std::string>(v, "indices_names", &indices_names);
  }

  /// The number of runs for each configuration at each iteration
  int nb_runs_by_iteration;

  /// The number of times the prior will be updated
  int nb_iterations;

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

  std::string conf_file(argv[1]);
  std::string data_path(argv[2]);
  std::string folder_name(rhoban_utils::getFormattedTime());
  std::default_random_engine engine(rhoban_random::getRandomEngine());
  int err = system(("mkdir -p " + folder_name).c_str());
  if (err != 0)
  {
    throw std::runtime_error(DEBUG_INFO + "Failed to create dir: '" + folder_name + "'");
  }
  std::ofstream results_file(folder_name + "/results.csv");

  // OPTIONAL: eventually, reduce number of columns if there is only 1 optimizer
  // or only 1 reader etc...
  results_file << "optimizer,model,reader,iteration_nb,trainingScore,validationScore,learningTime" << std::endl;

  // This should be inside the model loop.
  std::ostringstream name_oss;
  name_oss << "parameters.csv";
  std::ofstream params_file(folder_name + "/" + name_oss.str());

  Config conf;

  for (int iteration_nb = 0; iteration_nb < conf.nb_iterations; iteration_nb++)
  {
    std::cout << "iteration nb: " << iteration_nb << std::endl;
    conf.loadFile(conf_file);
    // For each model
    for (const auto& model_pair : conf.models)
    {
      std::string model_name = model_pair.first;
      const Model& model = *(model_pair.second);
      // Parameters file
      if (iteration_nb == 0)
      {
        std::vector<std::string> parameter_names = model.getParametersNames();
        params_file << "model,optimizer,reader,iteration_nb";
        for (size_t idx = 0; idx < parameter_names.size(); idx++)
        {
          params_file << "," << parameter_names[idx];
        };
        params_file << std::endl;
      }
      // For each optimizer

      for (const auto& optimizer_pair : conf.optimizers)
      {
        std::string optimizer_name = optimizer_pair.first;
        const Optimizer& optimizer = *(optimizer_pair.second);

        // Initialize the learning_model
        ModelLearner learner(model.clone(), conf.prior->clone(), conf.space->clone(), conf.predictor->clone(),
                             optimizer.clone(), model.getIndicesFromNames(conf.indices_names));
        // For each reader
        for (const auto& reader_pair : conf.readers)
        {
          std::string reader_name = reader_pair.first;
          const DataSetReader& reader = *(reader_pair.second);

          // Initializing parameters values management
          Eigen::VectorXd params_sum = Eigen::VectorXd::Zero(model.getParametersNames().size());
          std::vector<Eigen::VectorXd> training_parameters_values_per_run;
          for (size_t i = 0; i < learner.getTrainableIndices().size(); i++)
            training_parameters_values_per_run.push_back(Eigen::VectorXd::Zero(conf.nb_runs_by_iteration));

          // Perform multiple runs
          for (int run_id = 0; run_id < conf.nb_runs_by_iteration; run_id++)
          {
            // Print parameters space
            learner.getSpace().append(learner.getModel(), learner.getPrior(), learner.getTrainableIndices(), std::cout);

            // Extract data (splits between training and validation
            DataSet data = reader.extractSamples(data_path, &engine);
            // Learn model
            TimeStamp start = TimeStamp::now();
            ModelLearner::Result r = learner.learnParameters(data, &engine);
            TimeStamp end = TimeStamp::now();
            double learning_time = diffSec(start, end);
            // Writing scores
            results_file << optimizer_name << "," << model_name << "," << reader_name << "," << iteration_nb << ','
                         << r.training_log_likelihood << "," << r.validation_log_likelihood << "," << learning_time
                         << std::endl;
            // Writing params
            Eigen::VectorXd params = r.model->getParameters();
            params_file << model_name << "," << optimizer_name << "," << reader_name << "," << iteration_nb;
            for (int i = 0; i < params.rows(); i++)
            {
              params_file << "," << params(i);
            }
            params_sum += params;
            params_file << std::endl;

            int i = 0;
            for (int index : learner.getTrainableIndices())
            {
              training_parameters_values_per_run[i](run_id) = params[index];
              i++;
            }

            // Saving model params
            r.model->saveFile(folder_name + "/" + model_name + "_" + optimizer_name + "_" + reader_name + "_" +
                                  std::to_string(run_id) + "_trained_model.json",
                              true);

            // Writing prediction.
            learner.exportValidationResulstToCSV(
                *r.model, data,
                folder_name + "/predictionResultsTrained_" + model_name + "_" + optimizer_name + "_" + reader_name +
                    "_" + std::to_string(iteration_nb) + "_" + std::to_string(run_id) + ".csv",
                ',');
          }
          Eigen::VectorXd params_average = params_sum / conf.nb_runs_by_iteration;
          std::unique_ptr<Model> average_model = model.clone();
          average_model->setParameters(params_average);
          average_model->saveFile(folder_name + "/" + model_name + "_" + optimizer_name + "_" + reader_name + "_" +
                                      std::to_string(iteration_nb) + "_" + "_average_trained_models.json",
                                  true);

          // Updating prior
          std::unique_ptr<ModelPrior> new_prior = conf.prior->clone();
          if (conf.nb_runs_by_iteration > 1)
            new_prior->updateDeviations(learner.getModel(), training_parameters_values_per_run,
                                        learner.getTrainableIndices());

          // Writing conf
          std::cout << "copying conf." << std::endl;
          Config tmp_conf(conf);
          std::cout << "updating model." << std::endl;
          tmp_conf.models[model_name] = average_model->clone();
          std::cout << "updating prior." << std::endl;
          tmp_conf.prior = new_prior->clone();
          conf_file = folder_name + "/conf_" + std::to_string(iteration_nb) + ".json";
          std::cout << "writing conf file." << std::endl;
          tmp_conf.saveFile(conf_file);
        }
      }
    }
  }
}

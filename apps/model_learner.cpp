#include "rhoban_model_learning/model_learner.h"
#include "rhoban_model_learning/data_set_reader_factory.h"

#include "rhoban_random/tools.h"

#include <iostream>

using namespace rhoban_model_learning;

int main(int argc, char** argv)
{
  if (argc < 4)
  {
    std::cerr << "Usage: " << argv[0] << " <model_learner.json> <input_reader.json> <data_file>" << std::endl;
    exit(EXIT_FAILURE);
  }

  // Reading model learner from Json
  std::cout << "Read model learner from Json" << std::endl;
  ModelLearner learner;
  learner.loadFile(argv[1]);
  // Reading input reader from Json
  std::unique_ptr<DataSetReader> input_reader = DataSetReaderFactory().buildFromJsonFile(argv[2]);

  learner.getSpace().append(learner.getModel(), learner.getPrior(), learner.getTrainableIndices(), std::cout);

  // Analyze data
  std::cout << "Analyze data." << std::endl;
  std::default_random_engine engine = rhoban_random::getRandomEngine();
  DataSet data = input_reader->extractSamples(argv[3], &engine);

  learner.exportValidationResulstToCSV(learner.getModel(), data, "predictionResultsUntrained.csv", ',');
  // Learn model
  std::cout << "Learn model." << std::endl;
  ModelLearner::Result r = learner.learnParameters(data, &engine);

  // Output csv file with results
  std::cout << "training score   : " << r.training_log_likelihood << std::endl;
  std::cout << "validation score : " << r.validation_log_likelihood << std::endl;

  // Write parameters
  Eigen::VectorXd params = r.model->getParameters();
  std::vector<std::string> param_names = r.model->getParametersNames();
  for (size_t i = 0; i < param_names.size(); i++)
  {
    std::cout << param_names[i];
    if (i != param_names.size() - 1)
    {
      std::cout << ",";
    }
  }
  std::cout << std::endl;
  for (int i = 0; i < params.rows(); i++)
  {
    std::cout << params(i);
    if (i != params.rows() - 1)
    {
      std::cout << ",";
    }
  }
  std::cout << std::endl;

  // Save model in Json format
  r.model->saveFile("trained_model.json");

  // Save obsevation prediction results
  learner.exportValidationResulstToCSV(*r.model, data, "predictionResultsTrained.csv", ',');
}

#include "rhoban_model_learning/ball_models/position_sequence_reader.h"

namespace rhoban_model_learning
{

PositionSequenceReader::PositionSequenceReader() :
  high_threshold(0.6),
  low_threshold(0.1)
{
}

std::vector<PositionSequence>
PositionSequenceReader::readPositionSequences(const std::string & file_path) const {
  if (!speed_estimator) {
    throw std::logic_error(DEBUG_INFO + " speed estimator is null");
  }
  //TODO: implement
}

std::string PositionSequenceReader::getClassName() const {
  return "PositionSequenceReader";
}
Json::Value PositionSequenceReader::toJson() const  {
  Json::Value v;
  v["speed_estimator"] = speed_estimator.toJsonFactory();
  v["low_threshold"] = low_threshold;
  v["high_threshold"] = high_threshold;
  return v;
}
void PositionSequenceReader::fromJson(const Json::Value & v, const std::string & dir_name) {
  SpeedEstimatorFactory().tryRead(v,"speed_estimator",&speed_estimator);
  rhoban_utils::tryRead(v,"high_threshold", &high_threshold);
  rhoban_utils::tryRead(v,"low_threshold", &low_threshold);
}

}

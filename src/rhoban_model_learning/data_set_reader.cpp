#include <rhoban_model_learning/data_set_reader.h>
#include <rhoban_model_learning/data_set_reader_factory.h>

#include <rhoban_model_learning/tools.h>

namespace rhoban_model_learning
{
std::unique_ptr<DataSetReader> DataSetReader::clone() const
{
  Json::Value v = toJson();
  std::unique_ptr<DataSetReader> other = DataSetReaderFactory().build(getClassName());
  other->fromJson(v, "./");
  return other;
}

}  // namespace rhoban_model_learning

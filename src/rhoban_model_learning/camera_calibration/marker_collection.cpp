#include <rhoban_model_learning/camera_calibration/marker_collection.h>

#include <rhoban_utils/tables/string_table.h>
#include <rhoban_utils/util.h>

using rhoban_utils::StringTable;

namespace rhoban_model_learning
{
MarkerCollection::MarkerCollection()
{
}

void MarkerCollection::loadFile(const std::string& path)
{
  StringTable table = StringTable::buildFromFile(path);

  if (table.nbCols() != 4)
  {
    throw std::runtime_error(DEBUG_INFO + " unexpected number of columns in file '" + path + "'");
  }

  for (size_t row_idx = 0; row_idx < table.nbRows(); row_idx++)
  {
    std::map<std::string, std::string> row = table.getRow(row_idx);
    Eigen::Vector3d pos;
    int idx = 0;
    for (const std::string& name : { "world_x", "world_y", "world_z" })
    {
      pos(idx++) = std::stod(row.at(name));
    }
    int marker_id = std::stoi(row.at("marker_id"));
    if (count(marker_id) > 0)
    {
      throw std::runtime_error(DEBUG_INFO + " duplicated marker_id: '" + std::to_string(marker_id) + "'");
    }
    this->operator[](marker_id) = pos;
  }
}

void MarkerCollection::saveFile(const std::string& path) const
{
  std::map<std::string, std::vector<std::string>> data;
  for (const auto& entry : *this)
  {
    data["marker_id"].push_back(std::to_string(entry.first));
    data["world_x"].push_back(std::to_string(entry.second.x()));
    data["world_y"].push_back(std::to_string(entry.second.y()));
    data["world_z"].push_back(std::to_string(entry.second.z()));
  }
  StringTable table({ "marker_id", "world_x", "world_y", "world_z" }, data);
  table.writeFile(path);
}

}  // namespace rhoban_model_learning

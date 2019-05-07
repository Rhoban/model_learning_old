#include <rhoban_model_learning/camera_calibration/marker_seen_collection.h>

#include <rhoban_utils/tables/string_table.h>
#include <rhoban_utils/util.h>

using rhoban_utils::StringTable;

namespace rhoban_model_learning
{
MarkerSeenCollection::MarkerSeenCollection()
{
}

void MarkerSeenCollection::loadFile(const std::string& path)
{
  StringTable table = StringTable::buildFromFile(path);

  if (table.nbCols() != 4)
  {
    throw std::runtime_error(DEBUG_INFO + " unexpected number of columns in file '" + path + "'");
  }

  for (size_t row_idx = 0; row_idx < table.nbRows(); row_idx++)
  {
    std::map<std::string, std::string> row = table.getRow(row_idx);
    Eigen::Vector2d pos;
    int idx = 0;
    for (const std::string& name : { "img_x", "img_y" })
    {
      pos(idx++) = std::stod(row.at(name));
    }
    int img_id = std::stoi(row.at("img_id"));
    int marker_id = std::stoi(row.at("marker_id"));
    std::map<int, Eigen::Vector2d>& img_markers = this->operator[](img_id);
    if (img_markers.count(marker_id) > 0)
    {
      throw std::runtime_error(DEBUG_INFO + " duplicated marker_id: '" + std::to_string(marker_id) + "' at img '" +
                               std::to_string(img_id) + "'");
    }
    img_markers[marker_id] = pos;
  }
}

void MarkerSeenCollection::saveFile(const std::string& path) const
{
  std::map<std::string, std::vector<std::string>> data;
  for (const auto& img_entry : *this)
  {
    for (const auto& marker_entry : img_entry.second)
    {
      data["img_id"].push_back(std::to_string(img_entry.first));
      data["marker_id"].push_back(std::to_string(marker_entry.first));
      data["img_x"].push_back(std::to_string(marker_entry.second.x()));
      data["img_y"].push_back(std::to_string(marker_entry.second.y()));
    }
  }
  StringTable table({ "img_id", "marker_id", "img_x", "img_y" }, data);
  table.writeFile(path);
}

}  // namespace rhoban_model_learning

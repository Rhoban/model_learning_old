#pragma once

#include <Eigen/Core>

#include <map>

namespace rhoban_model_learning
{
/**
 * A collection of marker maps the name of the markers with their position in a 3d world
 *
 * Can be saved/loaded from a csv file with the following columns:
 * - marker_id, world_x, world_y, world_z
 */
class MarkerCollection : public std::map<int, Eigen::Vector3d>
{
public:
  MarkerCollection();

  void loadFile(const std::string& path);
  void saveFile(const std::string& path) const;
};

}  // namespace rhoban_model_learning

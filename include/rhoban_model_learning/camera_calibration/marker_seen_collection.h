#pragma once

#include <Eigen/Core>

#include <map>

namespace rhoban_model_learning
{
/**
 * A collection of marker seen in an image, indexed first by image_index and then by marker_id
 */
class MarkerSeenCollection : public std::map<int, std::map<int, Eigen::Vector2d>>
{
public:
  MarkerSeenCollection();

  void loadFile(const std::string& path);
  void saveFile(const std::string& path) const;
};

}  // namespace rhoban_model_learning

#include "rhoban_model_learning/tags/aruco_cube.h"

#include <rhoban_utils/util.h>

#include <iostream>

namespace rhoban_model_learning
{
ArucoCube::ArucoCube() : pose(), sheets()
{
  pose.setMode(PoseModel::Mode::Quaternion);
}

ArucoCube::ArucoCube(const ArucoCube& other) : pose(other.pose), sheets(other.sheets)
{
}

std::map<int, ArucoTag> ArucoCube::getMarkers() const
{
  std::map<int, ArucoTag> markers;
  for (const auto& sheet_entry : sheets)
  {
    const TagsSheet& sheet = sheet_entry.second;
    for (const auto& entry : sheet.getMarkers())
    {
      int marker_id = entry.first;
      if (markers.count(marker_id) > 0)
      {
        throw std::runtime_error(DEBUG_INFO + " duplicated marker " + std::to_string(marker_id));
      }
      markers[marker_id] = entry.second;
    }
  }
  return markers;
}

int ArucoCube::getParametersSize() const
{
  return pose.getParametersSize();
}

Eigen::VectorXd ArucoCube::getParameters() const
{
  return pose.getParameters();
}

void ArucoCube::setParameters(const Eigen::VectorXd& new_params)
{
  pose.setParameters(new_params);
  updateSheets();
}

std::vector<std::string> ArucoCube::getParametersNames() const
{
  return pose.getParametersNames();
}

Json::Value ArucoCube::toJson() const
{
  Json::Value v;
  v["side"] = side;
  v["pose"] = pose.toJson();
  for (const auto& entry : sheets)
  {
    int face_id = entry.first;
    v["sheets"][std::to_string(face_id)] = entry.second.toJson();
  }
  return v;
}

void ArucoCube::fromJson(const Json::Value& v, const std::string& dir_path)
{
  (void)dir_path;
  rhoban_utils::tryRead(v, "side", &side);
  if (v.isObject() && v.isMember("pose"))
  {
    pose.fromJson(v["pose"], dir_path);
  }
  if (v.isObject() && v.isMember("sheets"))
  {
    const Json::Value& sheets_v = v["sheets"];
    sheets.clear();
    for (Json::ValueConstIterator it = sheets_v.begin(); it != sheets_v.end(); it++)
    {
      const std::string& key = it.name();
      int key_idx = std::stoi(key);
      sheets[key_idx].fromJson(sheets_v[key], dir_path);
    }
  }
  updateSheets();
}

std::string ArucoCube::getClassName() const
{
  return "ArucoCube";
}

void ArucoCube::updateSheets()
{
  for (auto& entry : sheets)
  {
    int face_id = entry.first;
    if (face_id > 3 || face_id < 0)
    {
      throw std::runtime_error(DEBUG_INFO + "Invalid face id: " + std::to_string(face_id));
    }
    // Computing transformations
    Eigen::Quaterniond sheet_orientation =
        pose.getQuaternion() * Eigen::AngleAxisd(face_id * M_PI / 2, Eigen::Vector3d::UnitZ());
    Eigen::Vector3d center_in_cube = Eigen::Matrix3d(Eigen::AngleAxisd(face_id * M_PI / 2, Eigen::Vector3d::UnitZ())) *
                                     Eigen::Vector3d(side / 2, 0, 0);
    Eigen::Vector3d center_in_world = pose.getPosFromSelf(center_in_cube);
    // Updating sheets
    entry.second.setPose(center_in_world, sheet_orientation);
  }
}

}  // namespace rhoban_model_learning

#include "rhoban_model_learning/tags/tags_sheet.h"

#include <iostream>

namespace rhoban_model_learning
{
TagsSheet::TagsSheet()
  : pose(), marker_size(0.09), dy(0.12), dz(0.10), cols(2), rows(3), markers_ids({ 0, 1, 2, 3, 4, 5 })
{
  pose.setMode(PoseModel::Mode::Quaternion);
}

TagsSheet::TagsSheet(double marker_size, double dy, double dz, int cols, int rows, const PoseModel& pose,
                     const std::vector<int>& markers_ids)
  : pose(pose), marker_size(marker_size), dy(dy), dz(dz), cols(cols), rows(rows), markers_ids(markers_ids)
{
}

TagsSheet::TagsSheet(const TagsSheet& other)
  : pose(other.pose)
  , marker_size(other.marker_size)
  , dy(other.dy)
  , dz(other.dz)
  , cols(other.cols)
  , rows(other.rows)
  , markers_ids(other.markers_ids)
{
}

void TagsSheet::setPose(const Eigen::Vector3d& pos, const Eigen::Quaterniond& orientation)
{
  pose.setPosition(pos);
  pose.setOrientation(orientation);
}

std::map<int, ArucoTag> TagsSheet::getMarkers() const
{
  std::map<int, ArucoTag> markers;
  for (int idx = 0; idx < (cols * rows); idx++)
  {
    int marker_id = markers_ids[idx];
    double coeff_y(0), coeff_z(0);
    if (cols > 1)
    {
      int col = idx % cols;
      coeff_y = col - (cols - 1) / 2.0;
    }
    if (rows > 1)
    {
      int row = std::floor(idx / cols);
      coeff_z = row - (rows - 1) / 2.0;
    }
    Eigen::Vector3d marker_center_self(0, dy * coeff_y, dz * coeff_z);
    Eigen::Vector3d marker_center = pose.getPosFromSelf(marker_center_self);
    markers[marker_id] = ArucoTag(marker_id, marker_size, marker_center, pose.getQuaternion());
  }
  return markers;
}

int TagsSheet::getParametersSize() const
{
  return pose.getParametersSize();
}

Eigen::VectorXd TagsSheet::getParameters() const
{
  return pose.getParameters();
}

void TagsSheet::setParameters(const Eigen::VectorXd& new_params)
{
  pose.setParameters(new_params);
}

std::vector<std::string> TagsSheet::getParametersNames() const
{
  return pose.getParametersNames();
}

Json::Value TagsSheet::toJson() const
{
  Json::Value v;
  v["marker_size"] = marker_size;
  v["markers_ids"] = rhoban_utils::vector2Json(markers_ids);
  v["dy"] = dy;
  v["dz"] = dz;
  v["cols"] = cols;
  v["rows"] = rows;
  v["pose"] = pose.toJson();
  return v;
}

void TagsSheet::fromJson(const Json::Value& v, const std::string& dir_path)
{
  (void)dir_path;
  rhoban_utils::tryRead(v, "marker_size", &marker_size);
  rhoban_utils::tryRead(v, "dy", &dy);
  rhoban_utils::tryRead(v, "dz", &dz);
  rhoban_utils::tryRead(v, "cols", &cols);
  rhoban_utils::tryRead(v, "rows", &rows);
  rhoban_utils::tryReadVector(v, "markers_ids", &markers_ids);
  if (v.isObject() && v.isMember("pose"))
  {
    pose.fromJson(v["pose"], dir_path);
  }
}

std::string TagsSheet::getClassName() const
{
  return "TagsSheet";
}

}  // namespace rhoban_model_learning

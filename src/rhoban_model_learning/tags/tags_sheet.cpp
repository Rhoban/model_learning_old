#include "rhoban_model_learning/tags/tags_sheet.h"

#include <iostream>

namespace rhoban_model_learning
{
TagsSheet::TagsSheet()
  : sheet_pose(), marker_size(0.09), dy(0.12), dz(0.10), cols(2), rows(3), markers_ids({ 0, 1, 2, 3, 4, 5 })
{
}

TagsSheet::TagsSheet(double marker_size, double dy, double dz, int cols, int rows, const PoseModel& sheet_pose,
                     const std::vector<int>& markers_ids)
  : sheet_pose(sheet_pose), marker_size(marker_size), dy(dy), dz(dz), cols(cols), rows(rows), markers_ids(markers_ids)
{
}

TagsSheet::TagsSheet(const TagsSheet& other)
  : sheet_pose(other.sheet_pose)
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
  sheet_pose.pos = pos;
  sheet_pose.orientation = orientation;
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
    Eigen::Vector3d marker_center = sheet_pose.getPosFromSelf(marker_center_self);
    markers[marker_id] = ArucoTag(marker_id, marker_size, marker_center, sheet_pose.orientation);
  }
  return markers;
}

int TagsSheet::getParametersSize() const
{
  return sheet_pose.getParametersSize();
}

Eigen::VectorXd TagsSheet::getParameters() const
{
  return sheet_pose.getParameters();
}

void TagsSheet::setParameters(const Eigen::VectorXd& new_params)
{
  sheet_pose.setParameters(new_params);
}

std::vector<std::string> TagsSheet::getParametersNames() const
{
  return sheet_pose.getParametersNames();
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
  v["sheet_pose"] = sheet_pose.toJson();
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
  if (v.isObject() && v.isMember("sheet_pose"))
  {
    sheet_pose.fromJson(v["sheet_pose"], dir_path);
  }
}

std::string TagsSheet::getClassName() const
{
  return "TagsSheet";
}

}  // namespace rhoban_model_learning

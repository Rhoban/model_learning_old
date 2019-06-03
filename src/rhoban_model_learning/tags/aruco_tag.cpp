#include "rhoban_model_learning/tags/aruco_tag.h"

namespace rhoban_model_learning
{
ArucoTag::ArucoTag() : ArucoTag(-1, 0.1, Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond(Eigen::Vector4d(1, 0, 0, 0)))
{
}

ArucoTag::ArucoTag(int marker_id, double marker_size, const Eigen::Vector3d& marker_center,
                   const Eigen::Quaterniond& orientation)
  : marker_id(marker_id), marker_size(marker_size), marker_center(marker_center), orientation(orientation)
{
}

Eigen::Vector3d ArucoTag::getCorner(int corner_id) const
{
  double half_size = marker_size / 2;
  double x = -half_size + marker_size * (corner_id % 2);
  double y = -half_size + marker_size * (corner_id / 2);
  return orientation * (Eigen::Vector3d(x, y, 0) + marker_center);
}

}  // namespace rhoban_model_learning

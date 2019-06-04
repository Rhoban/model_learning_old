#include "rhoban_model_learning/tags/aruco_tag.h"
#include <rhoban_utils/util.h>

namespace rhoban_model_learning
{
ArucoTag::ArucoTag() : ArucoTag(-1, 0.1, Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond(Eigen::Vector4d(1, 0, 0, 0)))
{
}

ArucoTag::ArucoTag(int marker_id, double marker_size, const Eigen::Vector3d& marker_center,
                   const Eigen::Quaterniond& orientation)
  : marker_id(marker_id), marker_size(marker_size), marker_center(marker_center), orientation(orientation.normalized())
{
}

Eigen::Vector3d ArucoTag::getCorner(int corner_id) const
{
  if (corner_id < 0 || corner_id > 3)
  {
    throw std::runtime_error(DEBUG_INFO + "corner id should be 0, 1, 2 or 3.");
  }

  double half_size = marker_size / 2;

  double y;
  double z;
  switch (corner_id)
  {
    case 0:
      y = -half_size;
      z = half_size;
      break;
    case 1:
      y = half_size;
      z = half_size;
      break;
    case 2:
      y = half_size;
      z = -half_size;
      break;
    case 3:
      y = -half_size;
      z = -half_size;
      break;
  }
  return orientation * Eigen::Vector3d(0, y, z) + marker_center;
}

}  // namespace rhoban_model_learning

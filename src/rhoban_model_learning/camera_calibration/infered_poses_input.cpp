#include "rhoban_model_learning/camera_calibration/infered_poses_input.h"
#include <rhoban_utils/util.h>

namespace rhoban_model_learning
{
InferedPosesInput::InferedPosesInput() : aruco_id(-1)
{
}

InferedPosesInput::InferedPosesInput(std::vector<Eigen::Vector3d> tags_to_infer_, int aruco_id_)
  : tags_to_infer(tags_to_infer_), aruco_id(aruco_id_)
{
}

InferedPosesInput::InferedPosesInput(const InferedPosesInput& other)
  : tags_to_infer(other.tags_to_infer), aruco_id(other.aruco_id)
{
}

std::unique_ptr<Input> InferedPosesInput::clone() const
{
  return std::unique_ptr<Input>(new InferedPosesInput(*this));
}

}  // namespace rhoban_model_learning

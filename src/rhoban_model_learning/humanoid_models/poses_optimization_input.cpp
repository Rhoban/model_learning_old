#include "rhoban_model_learning/humanoid_models/poses_optimization_input.h"

namespace rhoban_model_learning
{
PosesOptimizationInput::PosesOptimizationInput() : image_id(-1), aruco_id(-1), corner_id(-1)
{
}

PosesOptimizationInput::PosesOptimizationInput(int image_id, int aruco_id, int corner_id,
                                               Eigen::Affine3d camera_from_self, Eigen::Affine3d camera_from_head_base)
  : image_id(image_id)
  , aruco_id(aruco_id)
  , corner_id(corner_id)
  , camera_from_self(camera_from_self)
  , camera_from_head_base(camera_from_head_base)
{
}

PosesOptimizationInput::PosesOptimizationInput(const PosesOptimizationInput& other)
  : image_id(other.image_id)
  , aruco_id(other.aruco_id)
  , corner_id(other.corner_id)
  , camera_from_self(other.camera_from_self)
  , camera_from_head_base(other.camera_from_head_base)
{
}

std::unique_ptr<Input> PosesOptimizationInput::clone() const
{
  return std::unique_ptr<Input>(new PosesOptimizationInput(*this));
}

}  // namespace rhoban_model_learning

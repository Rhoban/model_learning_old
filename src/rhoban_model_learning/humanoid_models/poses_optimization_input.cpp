#include "rhoban_model_learning/humanoid_models/poses_optimization_input.h"

namespace rhoban_model_learning
{
PosesOptimizationInput::PosesOptimizationInput() : image_id(-1), aruco_id(-1)
{
}

PosesOptimizationInput::PosesOptimizationInput(int image_id, int aruco_id, PoseModel camera_from_field)
  : image_id(image_id), aruco_id(aruco_id), camera_from_field(camera_from_field)
{
}

PosesOptimizationInput::PosesOptimizationInput(const PosesOptimizationInput& other)
  : image_id(other.image_id), aruco_id(other.aruco_id), camera_from_field(other.camera_from_field)
{
}

std::unique_ptr<Input> PosesOptimizationInput::clone() const
{
  return std::unique_ptr<Input>(new PosesOptimizationInput(*this));
}

}  // namespace rhoban_model_learning

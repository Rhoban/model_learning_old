#pragma once

#include "rhoban_model_learning/input.h"
#include "rhoban_model_learning/humanoid_models/pose_model.h"

namespace rhoban_model_learning
{
class PosesOptimizationInput : public Input
{
public:
  PosesOptimizationInput();
  PosesOptimizationInput(int image_id, int aruco_id, int corner_id, PoseModel camera_from_self,
                         PoseModel camera_from_head_base);
  PosesOptimizationInput(const PosesOptimizationInput& other);

  std::unique_ptr<Input> clone() const override;

  /// Image identifier
  int image_id;

  /// Marker identifier
  int aruco_id;

  /// Corner identifier
  int corner_id;

  /// Camera from self pose
  PoseModel camera_from_self;

  /// Head base from camera
  PoseModel camera_from_head_base;
};

}  // namespace rhoban_model_learning

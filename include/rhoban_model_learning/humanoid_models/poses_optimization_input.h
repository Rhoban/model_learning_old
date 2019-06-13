#pragma once

#include "rhoban_model_learning/input.h"
#include <Eigen/Geometry>

namespace rhoban_model_learning
{
class PosesOptimizationInput : public Input
{
public:
  PosesOptimizationInput();
  PosesOptimizationInput(int image_id, int aruco_id, int corner_id, Eigen::Affine3d camera_from_self,
                         Eigen::Affine3d camera_from_head_base);
  PosesOptimizationInput(const PosesOptimizationInput& other);

  std::unique_ptr<Input> clone() const override;

  /// Image identifier
  int image_id;

  /// Marker identifier
  int aruco_id;

  /// Corner identifier
  int corner_id;

  /// Camera from self pose
  Eigen::Affine3d camera_from_self;

  /// Head base from camera
  Eigen::Affine3d camera_from_head_base;
};

}  // namespace rhoban_model_learning

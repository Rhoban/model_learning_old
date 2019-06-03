#pragma once

#include "rhoban_model_learning/input.h"

#include <rhoban_utils/util.h>
#include <Eigen/Core>

namespace rhoban_model_learning
{
class InferedPosesInput : public Input
{
public:
  InferedPosesInput();
  InferedPosesInput(const InferedPosesInput& other);
  InferedPosesInput(std::vector<Eigen::Vector3d> tags_to_infer, int aruco_id);

  std::unique_ptr<Input> clone() const override;

  /// Tags used to infer the camera extrinsic parameters
  /// A tag is describre by the triplet (aruco_id, px, py)
  std::vector<Eigen::Vector3d> tags_to_infer;

  /// Marker identifier
  int aruco_id;
};

}  // namespace rhoban_model_learning

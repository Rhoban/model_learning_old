#pragma once

#include "rhoban_model_learning/input.h"
#include "rhoban_model_learning/basic_models/pose_model.h"

#include <rhoban_utils/util.h>
#include <Eigen/Core>

namespace rhoban_model_learning
{
class InferedPosesHumanoidInput : public Input
{
public:
  InferedPosesHumanoidInput();
  InferedPosesHumanoidInput(const InferedPosesHumanoidInput& other);
  InferedPosesHumanoidInput(std::vector<Eigen::Vector3d> tags_to_infer_sheet_world_pos,
                            PoseModel camera_world_pose_of_sheet, PoseModel camera_world_pose_of_observation,
                            int aruco_id);

  std::unique_ptr<Input> clone() const override;

  /// Tags used to infer the position of the sheet with respect to the camera
  /// A tag is describre by the triplet (aruco_id, px, py)
  std::vector<Eigen::Vector3d> tags_to_infer_sheet_world_pos;

  /// The camera pose
  PoseModel camera_world_pose_of_sheet;
  PoseModel camera_world_pose_of_observation;

  /// Marker identifier
  int aruco_id;
};

}  // namespace rhoban_model_learning

#include "rhoban_model_learning/humanoid_models/infered_poses_humanoid_input.h"
#include <rhoban_utils/util.h>

namespace rhoban_model_learning
{
InferedPosesHumanoidInput::InferedPosesHumanoidInput() : aruco_id(-1)
{
}

InferedPosesHumanoidInput::InferedPosesHumanoidInput(const InferedPosesHumanoidInput& other)
  : tags_to_infer_sheet_world_pos(other.tags_to_infer_sheet_world_pos)
  , camera_world_pose_of_sheet(other.camera_world_pose_of_sheet)
  , camera_world_pose_of_observation(other.camera_world_pose_of_observation)
  , aruco_id(other.aruco_id)
{
}

InferedPosesHumanoidInput::InferedPosesHumanoidInput(std::vector<Eigen::Vector3d> tags_to_infer_sheet_world_pos,
                                                     PoseModel camera_world_pose_of_sheet,
                                                     PoseModel camera_world_pose_of_observation, int aruco_id)
  : tags_to_infer_sheet_world_pos(tags_to_infer_sheet_world_pos)
  , camera_world_pose_of_sheet(camera_world_pose_of_sheet)
  , camera_world_pose_of_observation(camera_world_pose_of_observation)
  , aruco_id(aruco_id)
{
}

std::unique_ptr<Input> InferedPosesHumanoidInput::clone() const
{
  return std::unique_ptr<Input>(new InferedPosesHumanoidInput(*this));
}

}  // namespace rhoban_model_learning

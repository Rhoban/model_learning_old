#include "rhoban_model_learning/log_models/log_machine_input.h"
#include <rhoban_utils/util.h>

namespace rhoban_model_learning
{
LogMachineInput::LogMachineInput() : timestamp(0), label_position(Eigen::Vector3d::Zero())
{
}

LogMachineInput::LogMachineInput(double timestamp_, Eigen::Vector3d label_position_)
  : timestamp(timestamp_), label_position(label_position_)
{
}

LogMachineInput::LogMachineInput(const LogMachineInput& other)
  : timestamp(other.timestamp), label_position(other.label_position)
{
}

std::unique_ptr<Input> LogMachineInput::clone() const
{
  return std::unique_ptr<Input>(new LogMachineInput(*this));
}

}  // namespace rhoban_model_learning

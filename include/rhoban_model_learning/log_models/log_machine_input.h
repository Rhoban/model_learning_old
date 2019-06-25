#pragma once

#include "rhoban_model_learning/input.h"

#include <Eigen/Core>

namespace rhoban_model_learning
{
class LogMachineInput : public Input
{
public:
  LogMachineInput();
  LogMachineInput(double timestamp, Eigen::Vector3d label_position);
  LogMachineInput(const LogMachineInput& other);

  virtual std::unique_ptr<Input> clone() const override;

  double timestamp;

  Eigen::Vector3d label_position;
};

}  // namespace rhoban_model_learning

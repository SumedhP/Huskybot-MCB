#include "control_operator_interface.hpp"

namespace huskybot::control
{
float ControlOperatorInterface::getTurretYawInput() { return 0.0f; }

float ControlOperatorInterface::getTurretPitchInput() { return 0.0f; }

float ControlOperatorInterface::getChassisXInput() { return 0.0f; }

float ControlOperatorInterface::getChassisYInput() { return 0.0f; }

float ControlOperatorInterface::getChassisRotationInput() { return 0.0f; }

}  // namespace huskybot::control

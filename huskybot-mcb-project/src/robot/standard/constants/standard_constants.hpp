#pragma once

#include "control/control_operator_interface.hpp"

#include "chassis_constants.hpp"
#include "firing_system_constants.hpp"
#include "turret_constants.hpp"

namespace huskybot::standard::constants
{
constexpr control::ControlOperatorInterfaceConfig CONTROL_OPERATOR_INTERFACE_CONFIG = {
    .maxTranslationSpeed = 0.0f,
    .maxRotationSpeed = 0.0f,
    .maxTurretYawSpeed = 0.0f,
    .maxTurretPitchSpeed = 0.0f,
    .mouseYawSensitivity = 0.0f,
    .mousePitchSensitivity = 0.0f,
    .stickDeadzone = 0.00f,
};

}  // namespace huskybot::standard::constants

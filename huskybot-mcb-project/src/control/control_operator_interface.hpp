#pragma once

#include <cstdint>

#include "tap/algorithms/ramp.hpp"
#include "tap/drivers.hpp"

namespace huskybot::control
{
struct ControlOperatorInterfaceConfig
{
    /// Max Chassis translation speed, in m/s.
    float maxTranslationSpeed = 0.0f;
    /// Max Chassis rotation rate, in rad/s.
    float maxRotationSpeed = 0.0f;
    /// Max Turret yaw/pitch rate, in rad/s.
    float maxTurretYawSpeed = 0.0f;
    float maxTurretPitchSpeed = 0.0f;
    /// Turret rate, in rad/s, per count of mouse movement.
    float mouseYawSensitivity = 0.0f;
    float mousePitchSensitivity = 0.0f;
    /// Deadzone for stick input
    float stickDeadzone = 0.0f;
};

/**
 * Robot-centric keybindings for control inputs.
 *
 * Not intended for command bindings, rather to combine remote + keyboard inputs into a single
 * output.
 */
class ControlOperatorInterface
{
public:
    ControlOperatorInterface(tap::Drivers &drivers, const ControlOperatorInterfaceConfig &config);
    virtual ~ControlOperatorInterface() = default;

    /**
     * @return The desired turret yaw velocity, in radians/second.
     */
    virtual float getTurretYawInput();

    /**
     * @return The desired turret pitch velocity, in radians/second.
     */
    virtual float getTurretPitchInput();

    /**
     * @return The desired chassis forward velocity, in meters/second.
     */
    virtual float getChassisXInput();

    /**
     * @return The desired chassis lateral velocity, in meters/second.
     */
    virtual float getChassisYInput();

    /**
     * @return The desired chassis rotational velocity, in radians/second.
     */
    virtual float getChassisRotationInput();

private:
    float stickInput(tap::communication::serial::Remote::Channel channel) const;
    float keyInput(tap::communication::serial::Remote::Key positiveKey,
                    tap::communication::serial::Remote::Key negativeKey) const;

    float chassisInput(tap::communication::serial::Remote::Channel channel,
                       tap::communication::serial::Remote::Key positiveKey,
                       tap::communication::serial::Remote::Key negativeKey) const;

    tap::Drivers &drivers;
    ControlOperatorInterfaceConfig config;
};

}  // namespace huskybot::control

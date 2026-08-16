#include "control_operator_interface.hpp"

#include <cmath>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/clock.hpp"

using tap::algorithms::limitVal;
using tap::communication::serial::Remote;

namespace huskybot::control
{
ControlOperatorInterface::ControlOperatorInterface(
    tap::Drivers &drivers,
    const ControlOperatorInterfaceConfig &config)
    : drivers(drivers),
      config(config)
{
}

float ControlOperatorInterface::stickInput(Remote::Channel channel) const
{
    float input = drivers.remote.getChannel(channel);
    return std::abs(input) < config.stickDeadzone ? 0.0f : input;
}

float ControlOperatorInterface::keyInput(Remote::Key positiveKey, Remote::Key negativeKey) const
{
    return (drivers.remote.keyPressed(positiveKey) ? 1.0f : 0.0f) -
           (drivers.remote.keyPressed(negativeKey) ? 1.0f : 0.0f);
}

float ControlOperatorInterface::chassisInput(Remote::Channel channel,
                                              Remote::Key positiveKey,
                                              Remote::Key negativeKey) const
{
    return limitVal(
               stickInput(channel) + keyInput(positiveKey, negativeKey),
               -1.0f,
               1.0f);
}

float ControlOperatorInterface::getTurretYawInput()
{
    return stickInput(Remote::Channel::RIGHT_HORIZONTAL) * config.maxTurretYawSpeed +
           drivers.remote.getMouseX() * config.mouseYawSensitivity;
}

float ControlOperatorInterface::getTurretPitchInput()
{
    return stickInput(Remote::Channel::RIGHT_VERTICAL) * config.maxTurretPitchSpeed +
           drivers.remote.getMouseY() * config.mousePitchSensitivity;
}

float ControlOperatorInterface::getChassisXInput()
{
    return chassisInput(
               Remote::Channel::LEFT_VERTICAL,
               Remote::Key::W,
               Remote::Key::S) *
           config.maxTranslationSpeed;
}

float ControlOperatorInterface::getChassisYInput()
{
    return chassisInput(
               Remote::Channel::LEFT_HORIZONTAL,
               Remote::Key::A,
               Remote::Key::D) *
           config.maxTranslationSpeed;
}

float ControlOperatorInterface::getChassisRotationInput()
{
    return chassisInput(
               Remote::Channel::WHEEL,
               Remote::Key::Q,
               Remote::Key::E) *
           config.maxRotationSpeed;
}

}  // namespace huskybot::control

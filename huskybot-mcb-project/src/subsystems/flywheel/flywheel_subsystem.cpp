#include "flywheel_subsystem.hpp"

#include <algorithm>

namespace huskybot::subsystems::flywheel
{
FlywheelSubsystem::FlywheelSubsystem(
    tap::Drivers* drivers,
    tap::motor::MotorInterface& leftMotor,
    tap::motor::MotorInterface& rightMotor,
    const tap::algorithms::SmoothPidConfig& pidConfig)
    : Subsystem(drivers),
      leftMotor(leftMotor),
      rightMotor(rightMotor),
      leftPid(pidConfig),
      rightPid(pidConfig)
{
}

void FlywheelSubsystem::initialize()
{
    leftMotor.initialize();
    rightMotor.initialize();
}

void FlywheelSubsystem::refresh()
{
    uint32_t currentTime = tap::arch::clock::getTimeMicroseconds();
    float dt = (currentTime - lastRefreshTime) / 1e6f;
    lastRefreshTime = currentTime;

    float leftError = desiredSpeed - leftMotor.getEncoder()->getVelocity();
    float leftOutput = leftPid.runController(leftError, leftMotor.getEncoder()->getVelocity(), dt);
    leftMotor.setDesiredOutput(static_cast<int32_t>(leftOutput));

    float rightError = desiredSpeed - rightMotor.getEncoder()->getVelocity();
    float rightOutput =
        rightPid.runController(rightError, rightMotor.getEncoder()->getVelocity(), dt);
    rightMotor.setDesiredOutput(static_cast<int32_t>(rightOutput));
}

void FlywheelSubsystem::refreshSafeDisconnect()
{
    leftMotor.setDesiredOutput(0);
    rightMotor.setDesiredOutput(0);
    lastRefreshTime = tap::arch::clock::getTimeMicroseconds();
}

void FlywheelSubsystem::setDesiredSpeed(float speed) { desiredSpeed = speed; }

float FlywheelSubsystem::getDesiredSpeed() const { return desiredSpeed; }

float FlywheelSubsystem::getCurrentSpeed() const
{
    return std::min(leftMotor.getEncoder()->getVelocity(), rightMotor.getEncoder()->getVelocity());
}

bool FlywheelSubsystem::isOnline() const
{
    return leftMotor.isMotorOnline() && rightMotor.isMotorOnline();
}
}  // namespace huskybot::subsystems::flywheel

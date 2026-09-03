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
    float dt = deltaTime.getElapsedTime();

    float leftError = desiredSpeed - leftMotor.getEncoder()->getVelocity();
    float leftOutput = leftPid.runControllerDerivateError(leftError, dt);
    leftMotor.setDesiredOutput(static_cast<int32_t>(leftOutput));

    float rightError = desiredSpeed - rightMotor.getEncoder()->getVelocity();
    float rightOutput = rightPid.runControllerDerivateError(rightError, dt);
    rightMotor.setDesiredOutput(static_cast<int32_t>(rightOutput));
}

void FlywheelSubsystem::refreshSafeDisconnect()
{
    desiredSpeed = 0.0f;
    leftMotor.setDesiredOutput(0);
    rightMotor.setDesiredOutput(0);
    deltaTime.restart();
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

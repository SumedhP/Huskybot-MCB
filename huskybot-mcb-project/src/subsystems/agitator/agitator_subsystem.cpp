#include "agitator_subsystem.hpp"

#include "tap/algorithms/math_user_utils.hpp"

namespace huskybot::subsystems::agitator
{
AgitatorSubsystem::AgitatorSubsystem(
    tap::Drivers* drivers,
    tap::motor::MotorInterface& motor,
    const AgitatorConfig& config,
    const tap::algorithms::SmoothPidConfig& pidConfig)
    : Subsystem(drivers),
      motor(motor),
      config(config),
      pidController(pidConfig)
{
}

void AgitatorSubsystem::initialize() { motor.initialize(); }

void AgitatorSubsystem::refresh()
{
    float dt = deltaTime.update();

    float positionalError = desiredPosition - getCurrentValue();
    float currentVelocity = motor.getEncoder()->getVelocity();
    float output = pidController.runController(positionalError, currentVelocity, dt);
    motor.setDesiredOutput(static_cast<int32_t>(output));
}

void AgitatorSubsystem::refreshSafeDisconnect()
{
    desiredPosition = getCurrentValue();
    motor.setDesiredOutput(0);
    deltaTime.restart();
}

void AgitatorSubsystem::moveToNextPosition() { desiredPosition += config.setpointIncrement; }

float AgitatorSubsystem::getCurrentValue() const
{
    return motor.getEncoder()->getPosition().getUnwrappedValue();
}

bool AgitatorSubsystem::isOnline() const { return motor.isMotorOnline(); }

bool AgitatorSubsystem::atDesiredPosition() const
{
    return tap::algorithms::compareFloatClose(getCurrentValue(), desiredPosition, config.tolerance);
}
}  // namespace huskybot::subsystems::agitator
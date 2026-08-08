#include "flywheels_on_command.hpp"

namespace huskybot::subsystems::flywheel
{
FlywheelsOnCommand::FlywheelsOnCommand(FlywheelSubsystem& flywheelSubsystem, float desiredSpeed)
    : flywheel(flywheelSubsystem),
      desiredSpeed(desiredSpeed)
{
    addSubsystemRequirement(&flywheelSubsystem);
}

void FlywheelsOnCommand::initialize() { flywheel.setDesiredSpeed(desiredSpeed); }

void FlywheelsOnCommand::execute() {}

void FlywheelsOnCommand::end(bool) { flywheel.setDesiredSpeed(0.0f); }

bool FlywheelsOnCommand::isFinished() const { return false; }

bool FlywheelsOnCommand::isReady() { return flywheel.isOnline(); }
}  // namespace huskybot::subsystems::flywheel

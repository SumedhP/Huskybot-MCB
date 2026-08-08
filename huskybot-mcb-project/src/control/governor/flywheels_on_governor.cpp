#include "flywheels_on_governor.hpp"

namespace huskybot::control::governor
{
FlywheelsOnGovernor::FlywheelsOnGovernor(subsystems::flywheel::FlywheelSubsystem& flywheel)
    : flywheel(flywheel)
{
}

bool FlywheelsOnGovernor::isReady() { return flywheel.getCurrentSpeed() > SPEED_THRESHOLD; }

bool FlywheelsOnGovernor::isFinished() { return !isReady(); }
}  // namespace huskybot::control::governor

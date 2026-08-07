#pragma once

#include "tap/control/governor/command_governor_interface.hpp"

#include "subsystems/flywheel/flywheel_subsystem.hpp"

namespace huskybot::control::governor
{
/**
 * Governor that blocks a governed Command from running (or continuing to run) unless the
 * flywheels are actually spinning, i.e. the slower of the two measured flywheel speeds is
 * above SPEED_THRESHOLD.
 */
class FlywheelsOnGovernor : public tap::control::governor::CommandGovernorInterface
{
public:
    static constexpr float SPEED_THRESHOLD = 0.0f;

    explicit FlywheelsOnGovernor(subsystems::flywheel::FlywheelSubsystem& flywheel);

    bool isReady() override;

    bool isFinished() override;

private:
    subsystems::flywheel::FlywheelSubsystem& flywheel;
};
}  // namespace huskybot::control::governor

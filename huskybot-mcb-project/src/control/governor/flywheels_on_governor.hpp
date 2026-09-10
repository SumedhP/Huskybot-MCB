#pragma once

#include "tap/control/governor/command_governor_interface.hpp"

#include "subsystems/flywheel/flywheel_subsystem.hpp"

namespace huskybot::control::governor
{
/**
 * Governor that prevents a command from running unless the flywheels are actually spinning.
 */
class FlywheelsOnGovernor : public tap::control::governor::CommandGovernorInterface
{
public:
    static constexpr float SPEED_THRESHOLD = 0.0f;

    explicit FlywheelsOnGovernor(subsystems::flywheel::FlywheelSubsystem& flywheel);

    // Returns whether the slower of the two measured flywheel speeds is above `SPEED_THRESHOLD`.
    bool isReady() override;

    bool isFinished() override;

private:
    subsystems::flywheel::FlywheelSubsystem& flywheel;
};
}  // namespace huskybot::control::governor

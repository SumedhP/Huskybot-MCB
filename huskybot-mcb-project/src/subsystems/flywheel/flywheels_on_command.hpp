#pragma once

#include "tap/control/command.hpp"

#include "flywheel_subsystem.hpp"

namespace huskybot::subsystems::flywheel
{
/**
 * Command that drives both flywheel motors to a desired speed, set to 0 when the command ends.
 */
class FlywheelsOnCommand : public tap::control::Command
{
public:
    static constexpr float DEFAULT_DESIRED_SPEED = 0.0f;

    /**
     * @param flywheelSubsystem The flywheel subsystem to control.
     * @param desiredSpeed The speed to drive the flywheels to while this command is running.
     */
    FlywheelsOnCommand(
        FlywheelSubsystem& flywheelSubsystem,
        float desiredSpeed = DEFAULT_DESIRED_SPEED);

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    bool isReady() override;

    const char* getName() const override { return "Flywheels On Command"; }

private:
    FlywheelSubsystem& flywheel;
    float desiredSpeed;
};
}  // namespace huskybot::subsystems::flywheel

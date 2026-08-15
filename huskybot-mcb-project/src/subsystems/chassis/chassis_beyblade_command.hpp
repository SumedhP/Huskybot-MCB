#pragma once

#include "chassis_drive_command.hpp"

namespace huskybot::subsystems::chassis
{
/**
 * Spins the chassis at a fixed rate while still driving turret-relative.
 *
 * The spin is the priority: if the chassis falls short of the target rate (power limiting is the
 * usual culprit), the translation input is scaled down by the same fraction the rotation is short
 * by, freeing up the headroom the spin needs. Hitting the target rate leaves translation
 * untouched, and stalling completely stops translation entirely.
 */
class ChassisBeybladeCommand : public tap::control::Command
{
public:
    /**
     * @param chassisSubsystem The chassis to drive
     * @param operatorInterface The operator input to read translation from
     * @param transforms The transform manager, used to drive turret-relative
     * @param targetRotationRate The counter-clockwise rate to spin at, in radians/second
     */
    ChassisBeybladeCommand(
        ChassisSubsystem& chassisSubsystem,
        huskybot::control::ControlOperatorInterface& operatorInterface,
        const algorithms::transforms::TransformManager& transforms,
        float targetRotationRate);

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    bool isReady() override;

    const char* getName() const override { return "Chassis Beyblade Command"; }

private:
    ChassisSubsystem& chassis;
    huskybot::control::ControlOperatorInterface& operatorInterface;
    const algorithms::transforms::TransformManager& transforms;
    float targetRotationRate;
};
}  // namespace huskybot::subsystems::chassis

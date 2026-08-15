#pragma once

#include "tap/control/command.hpp"

#include "algorithms/transforms/transform_manager.hpp"
#include "control/control_operator_interface.hpp"

#include "chassis_subsystem.hpp"

namespace huskybot::subsystems::chassis
{
/**
 * Reads the operator's translation input and rotates it out of the turret frame and into the
 * chassis frame, so that "forward" always means "the way the barrel is pointing" no matter which
 * way the chassis happens to be facing.
 *
 * Shared by every chassis command that drives turret-relative. The returned velocity's `r` is left
 * at zero for the caller to fill in.
 */
ChassisVelocity getTurretRelativeTranslation(
    huskybot::control::ControlOperatorInterface& operatorInterface,
    const algorithms::transforms::TransformManager& transforms);

/**
 * Drives the chassis turret-relative, passing the operator's rotation input straight through.
 */
class ChassisDriveCommand : public tap::control::Command
{
public:
    ChassisDriveCommand(
        ChassisSubsystem& chassisSubsystem,
        huskybot::control::ControlOperatorInterface& operatorInterface,
        const algorithms::transforms::TransformManager& transforms);

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    bool isReady() override;

    const char* getName() const override { return "Chassis Drive Command"; }

private:
    ChassisSubsystem& chassis;
    huskybot::control::ControlOperatorInterface& operatorInterface;
    const algorithms::transforms::TransformManager& transforms;
};
}  // namespace huskybot::subsystems::chassis

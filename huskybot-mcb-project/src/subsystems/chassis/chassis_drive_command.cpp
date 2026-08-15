#include "chassis_drive_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"

namespace huskybot::subsystems::chassis
{
ChassisVelocity getTurretRelativeTranslation(
    huskybot::control::ControlOperatorInterface& operatorInterface,
    const algorithms::transforms::TransformManager& transforms)
{
    ChassisVelocity velocity{
        .x = operatorInterface.getChassisXInput(),
        .y = operatorInterface.getChassisYInput()};

    // The turret's yaw is exactly how far the turret frame is rotated from the chassis frame, so
    // rotating the input by it lands the operator's intent in the chassis frame.
    tap::algorithms::rotateVector(
        &velocity.x,
        &velocity.y,
        transforms.getChassisToTurret().getYaw());

    return velocity;
}

ChassisDriveCommand::ChassisDriveCommand(
    ChassisSubsystem& chassisSubsystem,
    huskybot::control::ControlOperatorInterface& operatorInterface,
    const algorithms::transforms::TransformManager& transforms)
    : chassis(chassisSubsystem),
      operatorInterface(operatorInterface),
      transforms(transforms)
{
    addSubsystemRequirement(&chassisSubsystem);
}

void ChassisDriveCommand::initialize() {}

void ChassisDriveCommand::execute()
{
    ChassisVelocity velocity = getTurretRelativeTranslation(operatorInterface, transforms);
    velocity.r = operatorInterface.getChassisRotationInput();
    chassis.setDesiredVelocity(velocity);
}

void ChassisDriveCommand::end(bool) { chassis.setDesiredVelocity(ChassisVelocity()); }

bool ChassisDriveCommand::isFinished() const { return false; }

bool ChassisDriveCommand::isReady() { return chassis.isOnline(); }
}  // namespace huskybot::subsystems::chassis

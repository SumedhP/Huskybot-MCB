#include "chassis_beyblade_command.hpp"

#include <algorithm>
#include <cmath>

namespace huskybot::subsystems::chassis
{
ChassisBeybladeCommand::ChassisBeybladeCommand(
    ChassisSubsystem& chassisSubsystem,
    huskybot::control::ControlOperatorInterface& operatorInterface,
    const algorithms::transforms::TransformManager& transforms,
    float targetRotationRate)
    : chassis(chassisSubsystem),
      operatorInterface(operatorInterface),
      transforms(transforms),
      targetRotationRate(targetRotationRate)
{
    addSubsystemRequirement(&chassisSubsystem);
}

void ChassisBeybladeCommand::initialize() {}

void ChassisBeybladeCommand::execute()
{
    ChassisVelocity velocity = getTurretRelativeTranslation(operatorInterface, transforms);

    const float achievedFraction =
        std::min(std::abs(chassis.getCurrentVelocity().r) / std::abs(targetRotationRate), 1.0f);
    velocity.x *= achievedFraction;
    velocity.y *= achievedFraction;
    velocity.r = targetRotationRate;

    chassis.setDesiredVelocity(velocity);
}

void ChassisBeybladeCommand::end(bool) { chassis.setDesiredVelocity(ChassisVelocity()); }

bool ChassisBeybladeCommand::isFinished() const { return false; }

bool ChassisBeybladeCommand::isReady() { return chassis.isOnline(); }
}  // namespace huskybot::subsystems::chassis

#include "transform_manager.hpp"

namespace huskybot::algorithms::transforms
{
TransformManager::TransformManager(const subsystems::turret::TurretSubsystem& turret)
    : turret(turret)
{
}

void TransformManager::update()
{
    const float turretChassisYaw = turret.getChassisFrameYaw().getWrappedValue();
    const float turretChassisPitch = turret.getChassisFramePitch().getWrappedValue();

    // The chassis' heading is turret world frame - turret chassis frame
    const float chassisYaw = turret.getWorldFrameYaw().getWrappedValue() - turretChassisYaw;
    const float chassisYawVelocity =
        turret.getWorldFrameYawVelocity() - turret.getChassisFrameYawVelocity();

    worldToChassis = Transform(
        DynamicPosition(),
        DynamicOrientation(0.0f, 0.0f, chassisYaw, 0.0f, 0.0f, chassisYawVelocity));

    chassisToTurret = Transform(
        DynamicPosition(),
        DynamicOrientation(
            0.0f,
            turretChassisPitch,
            turretChassisYaw,
            0.0f,
            turret.getChassisFramePitchVelocity(),
            turret.getChassisFrameYawVelocity()));

    worldToTurret = worldToChassis.compose(chassisToTurret);
}

const Transform& TransformManager::getWorldToChassis() const { return worldToChassis; }

const Transform& TransformManager::getWorldToTurret() const { return worldToTurret; }

const Transform& TransformManager::getChassisToTurret() const { return chassisToTurret; }
}  // namespace huskybot::algorithms::transforms

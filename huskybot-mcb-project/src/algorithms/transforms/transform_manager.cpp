#include "transform_manager.hpp"

namespace huskybot::algorithms::transforms
{
TransformManager::TransformManager(
    const subsystems::turret::TurretSubsystem& turret,
    const TransformManagerConfig& config)
    : turret(turret),
      config(config)
{
}

void TransformManager::update()
{
    const float turretChassisYaw = turret.getChassisFrameYaw().getWrappedValue();
    const float turretChassisPitch = turret.getChassisFramePitch().getWrappedValue();

    // The chassis' heading is whatever is left of the turret's world-frame heading once the
    // turret's rotation relative to the chassis is taken back out.
    const float chassisYaw = turret.getWorldFrameYaw().getWrappedValue() - turretChassisYaw;
    const float chassisYawVelocity =
        turret.getWorldFrameYawVelocity() - turret.getChassisFrameYawVelocity();

    worldToChassis = Transform(
        chassisPositionInWorld,
        DynamicOrientation(0.0f, 0.0f, chassisYaw, 0.0f, 0.0f, chassisYawVelocity));

    // The turret rotates in place about its mounting point, so the live encoder rotation composes
    // straight onto the fixed mounting transform.
    chassisToTurret = config.chassisToTurretMount.compose(DynamicOrientation(
        0.0f,
        turretChassisPitch,
        turretChassisYaw,
        0.0f,
        turret.getChassisFramePitchVelocity(),
        turret.getChassisFrameYawVelocity()));

    worldToTurret = worldToChassis.compose(chassisToTurret);
}

void TransformManager::setChassisOdometry(const DynamicPosition& chassisPositionInWorld)
{
    this->chassisPositionInWorld = chassisPositionInWorld;
}

const Transform& TransformManager::getWorldToChassis() const { return worldToChassis; }

const Transform& TransformManager::getWorldToTurret() const { return worldToTurret; }

const Transform& TransformManager::getChassisToTurret() const { return chassisToTurret; }
}  // namespace huskybot::algorithms::transforms

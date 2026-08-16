#pragma once

#include "tap/algorithms/transforms/transform.hpp"

#include "subsystems/turret/turret_subsystem.hpp"

namespace huskybot::algorithms::transforms
{
using namespace tap::algorithms::transforms;

/**
 * Tracks frames across the robot
 */
class TransformManager
{
public:
    /**
     * @param turret The turret subsystem to read yaw/pitch state from
     */
    TransformManager(const subsystems::turret::TurretSubsystem& turret);

    /**
     * Recomputes every transform from the system's current state.
     */
    void update();
    /**
     * @return The transform from the world frame to the chassis frame.
     */
    const Transform& getWorldToChassis() const;

    /**
     * @return The transform from the world frame to the turret frame.
     */
    const Transform& getWorldToTurret() const;

    /**
     * @return The transform from the chassis frame to the turret frame.
     */
    const Transform& getChassisToTurret() const;

private:
    const subsystems::turret::TurretSubsystem& turret;

    Transform worldToChassis;
    Transform worldToTurret;
    Transform chassisToTurret;
};

}  // namespace huskybot::algorithms::transforms

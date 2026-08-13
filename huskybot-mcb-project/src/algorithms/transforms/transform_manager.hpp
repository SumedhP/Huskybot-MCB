#pragma once

#include "tap/algorithms/transforms/transform.hpp"

#include "subsystems/turret/turret_subsystem.hpp"

namespace huskybot::algorithms::transforms
{
using namespace tap::algorithms::transforms;

struct TransformManagerConfig
{
    Transform chassisToTurretMount = Transform::identity();
};

/**
 * Tracks frames across the robot
 */
class TransformManager
{
public:
    /**
     * @param turret The turret subsystem to read yaw/pitch state from
     * @param config The transform configuration
     */
    TransformManager(
        const subsystems::turret::TurretSubsystem& turret,
        const TransformManagerConfig& config);

    /**
     * Recomputes every transform from the turret's current state and the most recently reported
     * chassis odometry. Call once per main loop iteration, after the turret's sensors have been
     * refreshed.
     */
    void update();

    /**
     * Reports where the chassis is in the world frame. Intended to be called by an external
     * odometry source; the transforms keep using the last reported value until it is called again.
     *
     * @param chassisPositionInWorld The chassis origin's position, velocity, and acceleration in
     * the world frame
     */
    void setChassisOdometry(const DynamicPosition& chassisPositionInWorld);

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
    TransformManagerConfig config;

    DynamicPosition chassisPositionInWorld;

    Transform worldToChassis;
    Transform worldToTurret;
    Transform chassisToTurret;
};

}  // namespace huskybot::algorithms::transforms

#pragma once

namespace huskybot::control
{
/**
 * Interface for operator input. Not yet wired to a real input device.
 */
class ControlOperatorInterface
{
public:
    /**
     * @return The desired turret yaw velocity, in radians/second.
     */
    virtual float getTurretYawInput();

    /**
     * @return The desired turret pitch velocity, in radians/second.
     */
    virtual float getTurretPitchInput();

    /**
     * @return The desired chassis forward velocity, in meters/second, relative to the turret.
     */
    virtual float getChassisXInput();

    /**
     * @return The desired chassis leftward velocity, in meters/second, relative to the turret.
     */
    virtual float getChassisYInput();

    /**
     * @return The desired chassis counter-clockwise rotational velocity, in radians/second.
     */
    virtual float getChassisRotationInput();
};

}  // namespace huskybot::control

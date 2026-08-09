#pragma once

namespace huskybot::control
{
/**
 * Interface for turret operator input. Not yet wired to a real input device.
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
};

}  // namespace huskybot::control

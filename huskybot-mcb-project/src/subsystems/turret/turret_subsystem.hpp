#pragma once

#include "tap/algorithms/wrapped_float.hpp"
#include "tap/communication/sensors/imu/imu_interface.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"
#include "tap/motor/motor_interface.hpp"

namespace huskybot::subsystems::turret
{
struct TurretMotorConfig
{
    /// The encoder angle (rad) that corresponds to the turret pointing straight forward
    /// relative to the chassis.
    float chassisFrameZeroOffset = 0.0f;
};

/**
 * Dumb turret subsystem: reads the yaw/pitch motor encoders and an IMU, and reports chassis-frame
 * and world-frame angle/velocity for each axis. Exposes raw motor output setters; no control loop
 * runs here, that logic lives entirely in external commands.
 */
class TurretSubsystem : public tap::control::Subsystem
{
public:
    /**
     * @param drivers A pointer to the Drivers instance
     * @param yawMotor The yaw axis motor
     * @param pitchMotor The pitch axis motor
     * @param imu The IMU mounted on the turret, used to report world-frame angles
     * @param yawConfig The yaw axis configuration
     * @param pitchConfig The pitch axis configuration
     */
    TurretSubsystem(
        tap::Drivers* drivers,
        tap::motor::MotorInterface& yawMotor,
        tap::motor::MotorInterface& pitchMotor,
        tap::communication::sensors::imu::ImuInterface& imu,
        const TurretMotorConfig& yawConfig,
        const TurretMotorConfig& pitchConfig);

    void initialize() override;

    void refresh() override;
    void refreshSafeDisconnect() override;

    /**
     * Sets the raw yaw motor output.
     */
    void setYawMotorOutput(int32_t output);

    /**
     * Sets the raw pitch motor output.
     */
    void setPitchMotorOutput(int32_t output);

    /**
     * @return The chassis-frame yaw angle, in radians.
     */
    tap::algorithms::WrappedFloat getChassisFrameYaw() const;

    /**
     * @return The chassis-frame yaw velocity, in radians/second.
     */
    float getChassisFrameYawVelocity() const;

    /**
     * @return The chassis-frame pitch angle, in radians.
     */
    tap::algorithms::WrappedFloat getChassisFramePitch() const;

    /**
     * @return The chassis-frame pitch velocity, in radians/second.
     */
    float getChassisFramePitchVelocity() const;

    /**
     * @return The world-frame yaw angle, in radians, read directly from the IMU.
     */
    tap::algorithms::WrappedFloat getWorldFrameYaw() const;

    /**
     * @return The world-frame yaw velocity, in radians/second, read directly from the IMU.
     */
    float getWorldFrameYawVelocity() const;

    /**
     * @return The world-frame pitch angle, in radians, read directly from the IMU.
     */
    tap::algorithms::WrappedFloat getWorldFramePitch() const;

    /**
     * @return The world-frame pitch velocity, in radians/second, read directly from the IMU.
     */
    float getWorldFramePitchVelocity() const;

    /**
     * @return True if both turret motors are online, false otherwise.
     */
    bool isOnline() const;

private:
    tap::motor::MotorInterface& yawMotor;
    tap::motor::MotorInterface& pitchMotor;
    tap::communication::sensors::imu::ImuInterface& imu;
    TurretMotorConfig yawConfig;
    TurretMotorConfig pitchConfig;
};

}  // namespace huskybot::subsystems::turret

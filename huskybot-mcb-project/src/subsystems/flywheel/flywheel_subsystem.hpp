#pragma once

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"
#include "tap/motor/motor_interface.hpp"

namespace huskybot::subsystems::flywheel
{
class FlywheelSubsystem : public tap::control::Subsystem
{
public:
/**
 * Constructs a FlywheelSubsystem with the given motors and PID configuration. Both motors
 * are driven to the same desired speed, each with its own velocity PID controller.
 *
 * @param drivers A pointer to the Drivers instance
 * @param leftMotor The left flywheel motor
 * @param rightMotor The right flywheel motor
 * @param pidConfig The velocity PID configuration shared by both motors
 */
    FlywheelSubsystem(
        tap::Drivers* drivers,
        tap::motor::MotorInterface& leftMotor,
        tap::motor::MotorInterface& rightMotor,
        const tap::algorithms::SmoothPidConfig& pidConfig);

    void initialize() override;

    void refresh() override;
    void refreshSafeDisconnect() override;

    /**
     * Sets the desired speed, shared by both flywheel motors.
     */
    void setDesiredSpeed(float speed);

    /**
     * @return The desired speed shared by both flywheel motors.
     */
    float getDesiredSpeed() const;

    /**
     * @return The measured speed of the slower of the two flywheel motors.
     */
    float getCurrentSpeed() const;

    /**
     * @return True if both flywheel motors are online, false otherwise.
     */
    bool isOnline() const;

private:
    tap::motor::MotorInterface& leftMotor;
    tap::motor::MotorInterface& rightMotor;
    tap::algorithms::SmoothPid leftPid;
    tap::algorithms::SmoothPid rightPid;
    float desiredSpeed = 0.0f;
    uint32_t lastRefreshTime = 0;
};

}  // namespace huskybot::subsystems::flywheel

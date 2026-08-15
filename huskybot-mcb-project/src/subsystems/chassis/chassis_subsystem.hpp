#pragma once

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"
#include "tap/motor/motor_interface.hpp"

#include "chassis_kinematics.hpp"

namespace huskybot::subsystems::chassis
{
/**
 * A four-wheel holonomic chassis. Every method speaks chassis-frame velocity; the wheel matrix
 * handed to the constructor is the only thing that knows whether the wheels are mecanum, omni, or
 * anything else holonomic, so callers never have to care which is bolted on.
 *
 * Each wheel runs its own velocity PID towards the speed the inverse kinematics asks for.
 */
class ChassisSubsystem : public tap::control::Subsystem
{
public:
    /**
     * Constructs a ChassisSubsystem with the given motors, kinematics, and PID configuration.
     *
     * @param drivers A pointer to the Drivers instance
     * @param leftFrontMotor The left front wheel motor
     * @param rightFrontMotor The right front wheel motor
     * @param leftBackMotor The left back wheel motor
     * @param rightBackMotor The right back wheel motor
     * @param wheelMatrix The inverse kinematics for this chassis, see `chassis_kinematics.hpp`
     * @param pidConfig The velocity PID configuration shared by all four wheels
     */
    ChassisSubsystem(
        tap::Drivers* drivers,
        tap::motor::MotorInterface& leftFrontMotor,
        tap::motor::MotorInterface& rightFrontMotor,
        tap::motor::MotorInterface& leftBackMotor,
        tap::motor::MotorInterface& rightBackMotor,
        const WheelMatrix& wheelMatrix,
        const tap::algorithms::SmoothPidConfig& pidConfig);

    void initialize() override;

    void refresh() override;
    void refreshSafeDisconnect() override;

    /**
     * Sets the velocity the chassis should drive at, in the chassis frame.
     */
    void setDesiredVelocity(const ChassisVelocity& velocity);

    /**
     * @return The chassis-frame velocity most recently commanded. Not what the chassis is actually
     * doing; see `getCurrentVelocity` for that.
     */
    ChassisVelocity getDesiredVelocity() const;

    /**
     * @return The chassis-frame velocity the wheel encoders say the chassis is actually moving at.
     * A four-wheel chassis is overdetermined, so this is the least-squares best fit to the four
     * measured wheel speeds.
     */
    ChassisVelocity getCurrentVelocity() const;

    /**
     * @return True if all four wheel motors are online, false otherwise.
     */
    bool isOnline() const;

private:
    tap::motor::MotorInterface* motors[NUM_WHEELS];
    tap::algorithms::SmoothPid wheelPids[NUM_WHEELS];

    /// Chassis-frame velocity to wheel speeds.
    WheelMatrix wheelMatrix;
    /// Wheel speeds back to a chassis-frame velocity, the pseudoinverse of `wheelMatrix`.
    tap::algorithms::CMSISMat<3, NUM_WHEELS> chassisMatrix;

    ChassisVelocity desiredVelocity;
    uint32_t lastRefreshTime = 0;
};

}  // namespace huskybot::subsystems::chassis

#pragma once

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"
#include "tap/motor/motor_interface.hpp"

namespace huskybot::subsystems::agitator
{
struct AgitatorConfig
{
    float setpointIncrement = 0.0f;  /// Angle to rotate per click.
    float tolerance = 0.0f;          /// The tolerance for the agitator's position.
};

class AgitatorSubsystem : public tap::control::Subsystem
{
public:
/**
 * Constructs an AgitatorSubsystem with the given motor, drivers, and PID configuration.
 * 
 * @param drivers A pointer to the Drivers instance
 * @param motor The motor to use
 * @param config The configuration for the agitator
 * @param pidConfig The configuration for the PID controller
 */
    AgitatorSubsystem(
        tap::Drivers* drivers,
        tap::motor::MotorInterface& motor,
        const AgitatorConfig& config,
        const tap::algorithms::SmoothPidConfig& pidConfig);

    void initialize() override;

    void refresh() override;
    void refreshSafeDisconnect() override;

    /**
     * Increments the desired position of the agitator
     */
    void moveToNextPosition();

    /**
     * @return The current position of the agitator in cummulative revolutions, unwrapped.
     */
    float getCurrentValue() const;

    /**
     * @return True if the agitator motor is online, false otherwise.
     */
    bool isOnline() const;

    bool atDesiredPosition() const;

private:
    tap::motor::MotorInterface& motor;
    AgitatorConfig config;
    tap::algorithms::SmoothPid pidController;
    float desiredPosition = 0.0f;
    uint32_t lastRefreshTime = 0;
};

}  // namespace huskybot::subsystems::agitator

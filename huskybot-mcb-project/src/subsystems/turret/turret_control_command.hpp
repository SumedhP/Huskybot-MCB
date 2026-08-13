#pragma once

#include "tap/algorithms/wrapped_float.hpp"
#include "tap/control/command.hpp"

#include "algorithms/controllers/cascade_pid_controller.hpp"
#include "algorithms/controllers/gravity_compensator.hpp"
#include "control/control_operator_interface.hpp"

#include "turret_subsystem.hpp"

namespace huskybot::subsystems::turret
{
/**
 * Drives the turret in world frame towards an operator-controlled setpoint, using a cascade PID
 * controller per axis with the operator's input velocity as a feedforward term. The pitch axis
 * additionally gets an open-loop gravity compensation term added to its output.
 */
class TurretControlCommand : public tap::control::Command
{
public:
    TurretControlCommand(
        TurretSubsystem& turretSubsystem,
        huskybot::control::ControlOperatorInterface& operatorInterface,
        algorithms::controllers::CascadePidController& yawController,
        algorithms::controllers::CascadePidController& pitchController,
        algorithms::controllers::GravityCompensator& pitchGravityCompensator);

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    bool isReady() override;

    const char* getName() const override { return "Turret Control Command"; }

private:
    TurretSubsystem& turret;
    huskybot::control::ControlOperatorInterface& operatorInterface;
    algorithms::controllers::CascadePidController& yawController;
    algorithms::controllers::CascadePidController& pitchController;
    algorithms::controllers::GravityCompensator& pitchGravityCompensator;

    tap::algorithms::WrappedFloat yawSetpoint = tap::algorithms::Angle(0.0f);
    tap::algorithms::WrappedFloat pitchSetpoint = tap::algorithms::Angle(0.0f);
    uint32_t lastExecuteTime = 0;
};
}  // namespace huskybot::subsystems::turret

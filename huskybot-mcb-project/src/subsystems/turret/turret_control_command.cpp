#include "turret_control_command.hpp"

#include "tap/architecture/clock.hpp"

namespace huskybot::subsystems::turret
{
TurretControlCommand::TurretControlCommand(
    TurretSubsystem& turretSubsystem,
    huskybot::control::ControlOperatorInterface& operatorInterface,
    algorithms::controllers::CascadePidController& yawController,
    algorithms::controllers::CascadePidController& pitchController,
    algorithms::controllers::GravityCompensator& pitchGravityCompensator)
    : turret(turretSubsystem),
      operatorInterface(operatorInterface),
      yawController(yawController),
      pitchController(pitchController),
      pitchGravityCompensator(pitchGravityCompensator)
{
    addSubsystemRequirement(&turretSubsystem);
}

void TurretControlCommand::initialize()
{
    yawSetpoint = turret.getWorldFrameYaw();
    pitchSetpoint = turret.getWorldFramePitch();
    lastExecuteTime = tap::arch::clock::getTimeMicroseconds();
}

void TurretControlCommand::execute()
{
    uint32_t currentTime = tap::arch::clock::getTimeMicroseconds();
    float dt = (currentTime - lastExecuteTime) / 1e6f;
    lastExecuteTime = currentTime;

    float yawInput = operatorInterface.getTurretYawInput();
    yawSetpoint += yawInput * dt;
    float yawOutput = yawController.runController(
        yawSetpoint,
        turret.getWorldFrameYaw(),
        turret.getWorldFrameYawVelocity(),
        yawInput,
        dt);
    turret.setYawMotorOutput(static_cast<int32_t>(yawOutput));

    float pitchInput = operatorInterface.getTurretPitchInput();
    pitchSetpoint += pitchInput * dt;
    tap::algorithms::WrappedFloat measuredPitch = turret.getWorldFramePitch();
    float pitchOutput = pitchController.runController(
        pitchSetpoint,
        measuredPitch,
        turret.getWorldFramePitchVelocity(),
        pitchInput,
        dt);
    pitchOutput += pitchGravityCompensator.calculateEffort(measuredPitch.getWrappedValue());
    turret.setPitchMotorOutput(static_cast<int32_t>(pitchOutput));
}

void TurretControlCommand::end(bool)
{
    turret.setYawMotorOutput(0);
    turret.setPitchMotorOutput(0);
}

bool TurretControlCommand::isFinished() const { return false; }

bool TurretControlCommand::isReady() { return turret.isOnline(); }
}  // namespace huskybot::subsystems::turret

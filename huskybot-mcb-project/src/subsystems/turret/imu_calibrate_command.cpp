#include "imu_calibrate_command.hpp"

#include <cmath>

#include "tap/architecture/clock.hpp"

using tap::communication::sensors::imu::ImuInterface;

namespace huskybot::subsystems::turret
{
ImuCalibrateCommand::ImuCalibrateCommand(
    TurretSubsystem& turretSubsystem,
    tap::communication::sensors::imu::AbstractIMU& imu,
    algorithms::controllers::CascadePidController& yawController,
    algorithms::controllers::CascadePidController& pitchController,
    const ImuCalibrateConfig& config)
    : turret(turretSubsystem),
      imu(imu),
      yawController(yawController),
      pitchController(pitchController),
      config(config)
{
    addSubsystemRequirement(&turretSubsystem);
}

void ImuCalibrateCommand::initialize()
{
    state = State::SETTLING;
    timeout.restart(config.settleTimeout);
    yawSetpoint = turret.getChassisFrameYaw();
    deltaTime.restart();
}

void ImuCalibrateCommand::execute()
{
    float dt = deltaTime.getElapsedTime();

    float yawOutput = yawController.runController(
        yawSetpoint,
        turret.getChassisFrameYaw(),
        turret.getChassisFrameYawVelocity(),
        0.0f,
        dt);
    turret.setYawMotorOutput(static_cast<int32_t>(yawOutput));

    float pitchOutput = pitchController.runController(
        tap::algorithms::Angle(config.levelPitch),
        turret.getChassisFramePitch(),
        turret.getChassisFramePitchVelocity(),
        0.0f,
        dt);
    turret.setPitchMotorOutput(static_cast<int32_t>(pitchOutput));

    if (state == State::SETTLING && (turretIsSettled() || timeout.isExpired()))
    {
        imu.requestCalibration();
        state = State::CALIBRATING;
        timeout.restart(config.calibrationTimeout);
    }
}

bool ImuCalibrateCommand::turretIsSettled() const
{
    return std::abs(turret.getChassisFrameYawVelocity()) < config.velocityTolerance &&
           std::abs(turret.getChassisFramePitchVelocity()) < config.velocityTolerance &&
           std::abs(turret.getChassisFramePitch().minDifference(
               tap::algorithms::Angle(config.levelPitch))) < config.positionTolerance &&
           std::abs(turret.getChassisFrameYaw().minDifference(yawSetpoint)) <
               config.positionTolerance;
}

void ImuCalibrateCommand::end(bool)
{
    turret.setYawMotorOutput(0);
    turret.setPitchMotorOutput(0);
}

bool ImuCalibrateCommand::isFinished() const
{
    // Give up on the timeout rather than hanging onto the turret forever; a disconnected IMU
    // never leaves IMU_CALIBRATING and would otherwise deadlock the turret.
    return state == State::CALIBRATING &&
           (imu.getImuState() == ImuInterface::ImuState::IMU_CALIBRATED || timeout.isExpired());
}

bool ImuCalibrateCommand::isReady() { return turret.isOnline(); }
}  // namespace huskybot::subsystems::turret

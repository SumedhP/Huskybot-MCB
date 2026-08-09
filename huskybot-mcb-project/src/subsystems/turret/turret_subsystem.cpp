#include "turret_subsystem.hpp"

namespace huskybot::subsystems::turret
{
TurretSubsystem::TurretSubsystem(
    tap::Drivers* drivers,
    tap::motor::MotorInterface& yawMotor,
    tap::motor::MotorInterface& pitchMotor,
    tap::communication::sensors::imu::ImuInterface& imu,
    const TurretMotorConfig& yawConfig,
    const TurretMotorConfig& pitchConfig)
    : Subsystem(drivers),
      yawMotor(yawMotor),
      pitchMotor(pitchMotor),
      imu(imu),
      yawConfig(yawConfig),
      pitchConfig(pitchConfig)
{
}

void TurretSubsystem::initialize()
{
    yawMotor.initialize();
    pitchMotor.initialize();
}

void TurretSubsystem::refresh() {}

void TurretSubsystem::refreshSafeDisconnect()
{
    yawMotor.setDesiredOutput(0);
    pitchMotor.setDesiredOutput(0);
}

void TurretSubsystem::setYawMotorOutput(int32_t output) { yawMotor.setDesiredOutput(output); }

void TurretSubsystem::setPitchMotorOutput(int32_t output) { pitchMotor.setDesiredOutput(output); }

tap::algorithms::WrappedFloat TurretSubsystem::getChassisFrameYaw() const
{
    return yawMotor.getEncoder()->getPosition() - yawConfig.chassisFrameZeroOffset;
}

float TurretSubsystem::getChassisFrameYawVelocity() const
{
    return yawMotor.getEncoder()->getVelocity();
}

tap::algorithms::WrappedFloat TurretSubsystem::getChassisFramePitch() const
{
    return pitchMotor.getEncoder()->getPosition() - pitchConfig.chassisFrameZeroOffset;
}

float TurretSubsystem::getChassisFramePitchVelocity() const
{
    return pitchMotor.getEncoder()->getVelocity();
}

tap::algorithms::WrappedFloat TurretSubsystem::getWorldFrameYaw() const
{
    return tap::algorithms::Angle(imu.getYaw());
}

float TurretSubsystem::getWorldFrameYawVelocity() const { return imu.getGz(); }

tap::algorithms::WrappedFloat TurretSubsystem::getWorldFramePitch() const
{
    return tap::algorithms::Angle(imu.getPitch());
}

float TurretSubsystem::getWorldFramePitchVelocity() const { return imu.getGy(); }

bool TurretSubsystem::isOnline() const
{
    return yawMotor.isMotorOnline() && pitchMotor.isMotorOnline();
}
}  // namespace huskybot::subsystems::turret

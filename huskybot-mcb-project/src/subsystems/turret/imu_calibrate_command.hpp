#pragma once

#include "tap/algorithms/wrapped_float.hpp"
#include "tap/architecture/timeout.hpp"
#include "tap/communication/sensors/imu/abstract_imu.hpp"
#include "tap/control/command.hpp"

#include "algorithms/controllers/cascade_pid_controller.hpp"
#include "util/delta_time.hpp"

#include "turret_subsystem.hpp"

namespace huskybot::subsystems::turret
{
struct ImuCalibrateConfig
{
    /// The chassis-frame pitch (rad) that puts the turret-mounted IMU level. Nonzero if the IMU
    /// is mounted at an angle to the pitch axis.
    float levelPitch = 0.0f;
    /// How far (rad) off level the turret may sit and still count as settled.
    float positionTolerance = 0.02f;
    /// How fast (rad/s) either axis may still be moving and count as settled.
    float velocityTolerance = 0.05f;
    /// How long (ms) to keep trying to settle before calibrating anyway. The IMU drifts until it
    /// has been calibrated at least once, so a mediocre calibration beats none at all.
    uint32_t settleTimeout = 5000;
    /// How long (ms) to wait for the IMU to report itself calibrated before giving up.
    uint32_t calibrationTimeout = 6000;
};

/**
 * Holds the turret level and still, then calibrates the turret-mounted IMU.
 *
 * The IMU's accelerometer offsets are only correct if it is level while sampling, and its gyro
 * offsets are only correct if it is stationary, so this command has to own the turret for the
 * duration. Feedback is taken from the encoders rather than the IMU: the IMU reports zeros while
 * calibrating, so driving off world-frame angles here would be a loop closed on nothing.
 */
class ImuCalibrateCommand : public tap::control::Command
{
public:
    ImuCalibrateCommand(
        TurretSubsystem& turretSubsystem,
        tap::communication::sensors::imu::AbstractIMU& imu,
        algorithms::controllers::CascadePidController& yawController,
        algorithms::controllers::CascadePidController& pitchController,
        const ImuCalibrateConfig& config);

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    bool isReady() override;

    const char* getName() const override { return "IMU Calibrate Command"; }

private:
    /// True once both axes are within tolerance of level and have stopped moving.
    bool turretIsSettled() const;

    TurretSubsystem& turret;
    tap::communication::sensors::imu::AbstractIMU& imu;
    algorithms::controllers::CascadePidController& yawController;
    algorithms::controllers::CascadePidController& pitchController;
    ImuCalibrateConfig config;

    enum class State
    {
        /// Driving the turret to level and waiting for it to stop moving.
        SETTLING,
        /// Calibration has been requested; holding the turret still until the IMU reports done.
        CALIBRATING,
    };

    State state = State::SETTLING;
    tap::arch::MilliTimeout timeout;

    /// Where the yaw axis was when the command started. Yaw doesn't affect how level the IMU is,
    /// so it is held where it was found rather than swung to a fixed heading.
    tap::algorithms::WrappedFloat yawSetpoint = tap::algorithms::Angle(0.0f);
    huskybot::util::DeltaTime deltaTime;
};
}  // namespace huskybot::subsystems::turret

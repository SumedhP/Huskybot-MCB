#pragma once

#include "tap/communication/can/can_bus.hpp"
#include "tap/motor/dji_motor.hpp"

#include "algorithms/controllers/cascade_pid_controller.hpp"
#include "algorithms/controllers/gravity_compensator.hpp"
#include "subsystems/turret/imu_calibrate_command.hpp"
#include "subsystems/turret/turret_subsystem.hpp"

namespace huskybot::standard::constants
{
using tap::algorithms::SmoothPidConfig;
using tap::can::CanBus;
using tap::motor::DjiMotor;
using tap::motor::MotorId;

constexpr CanBus TURRET_CAN_BUS = CanBus::CAN_BUS2;
constexpr MotorId YAW_MOTOR_ID = MotorId::MOTOR5;
constexpr MotorId PITCH_MOTOR_ID = MotorId::MOTOR6;

constexpr subsystems::turret::TurretMotorConfig YAW_MOTOR_CONFIG = {.chassisFrameZeroOffset = 0.0f};
constexpr subsystems::turret::TurretMotorConfig PITCH_MOTOR_CONFIG = {
    .chassisFrameZeroOffset = 0.0f};

constexpr algorithms::controllers::CascadePidControllerConfig YAW_PID_CONFIG = {
    .positionPidConfig = {.kp = 0.0f, .maxOutput = 20.0f},
    .velocityPidConfig = {.kp = 0.0f, .maxOutput = DjiMotor::MAX_OUTPUT_GM6020_mA},
    .feedforwardGain = 0.0f,
};

constexpr algorithms::controllers::CascadePidControllerConfig PITCH_PID_CONFIG = {
    .positionPidConfig = {.kp = 0.0f, .maxOutput = 20.0f},
    .velocityPidConfig = {.kp = 0.0f, .maxOutput = DjiMotor::MAX_OUTPUT_GM6020_mA},
    .feedforwardGain = 0.0f,
};

constexpr algorithms::controllers::GravityCompensatorConfig PITCH_GRAVITY_CONFIG = {
    .cgX = 0.0f,
    .cgZ = 0.0f,
    .gravityCompensationScalar = 0.0f,
};

constexpr algorithms::controllers::CascadePidControllerConfig IMU_CALIBRATE_YAW_PID_CONFIG = {
    .positionPidConfig = {.kp = 0.0f, .maxOutput = 20.0f},
    .velocityPidConfig = {.kp = 0.0f, .maxOutput = DjiMotor::MAX_OUTPUT_GM6020_mA},
    .feedforwardGain = 0.0f,
};

constexpr algorithms::controllers::CascadePidControllerConfig IMU_CALIBRATE_PITCH_PID_CONFIG = {
    .positionPidConfig = {.kp = 0.0f, .maxOutput = 20.0f},
    .velocityPidConfig = {.kp = 0.0f, .maxOutput = DjiMotor::MAX_OUTPUT_GM6020_mA},
    .feedforwardGain = 0.0f,
};

constexpr subsystems::turret::ImuCalibrateConfig IMU_CALIBRATE_CONFIG = {
    .levelPitch = 0.0f,
    .positionTolerance = 0.02f,
    .velocityTolerance = 0.05f,
    .settleTimeout = 5000,
    .calibrationTimeout = 6000,
};

}  // namespace huskybot::standard::constants
#pragma once

#include "tap/communication/can/can_bus.hpp"
#include "tap/motor/dji_motor.hpp"

#include "modm/math/geometry/angle.hpp"
#include "subsystems/agitator/agitator_subsystem.hpp"

namespace huskybot::standard::constants
{
using tap::algorithms::SmoothPidConfig;
using tap::can::CanBus;
using tap::motor::DjiMotor;
using tap::motor::MotorId;

/// Flywheel
constexpr CanBus FLYWHEEL_CAN_BUS = CanBus::CAN_BUS2;
constexpr MotorId LEFT_FLYWHEEL_MOTOR_ID = MotorId::MOTOR1;
constexpr MotorId RIGHT_FLYWHEEL_MOTOR_ID = MotorId::MOTOR2;

constexpr SmoothPidConfig FLYWHEEL_PID_CONFIG = {
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = DjiMotor::MAX_OUTPUT_820R,
};

// Target speed (in rad/s) to spin up to when firing
constexpr float FLYWHEEL_FIRING_SPEED = 0.0f;

/// Agitator
constexpr CanBus AGITATOR_CAN_BUS = CanBus::CAN_BUS2;
constexpr MotorId AGITATOR_MOTOR_ID = MotorId::MOTOR3;

constexpr float NUM_SLOTS = 8;
constexpr float ONE_POCKET_ANGLE = M_TWOPI / NUM_SLOTS;

constexpr subsystems::agitator::AgitatorConfig AGITATOR_CONFIG = {
    .setpointIncrement = ONE_POCKET_ANGLE,
    .tolerance = ONE_POCKET_ANGLE * 0.05f,  // 5% angular tolerance
};

constexpr SmoothPidConfig AGITATOR_PID_CONFIG = {
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = DjiMotor::MAX_OUTPUT_C620,
};

constexpr float PROJECTILE_HEAT_COST = algorithms::heat::HeatPredictor::HEAT_COST_17MM;

}  // namespace huskybot::standard::constants
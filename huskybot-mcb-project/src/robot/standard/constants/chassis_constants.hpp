#pragma once

#include "tap/communication/can/can_bus.hpp"
#include "tap/motor/dji_motor.hpp"

#include "algorithms/controllers/cascade_pid_controller.hpp"
#include "modm/math/geometry/angle.hpp"
#include "subsystems/chassis/chassis_kinematics.hpp"

namespace huskybot::standard::constants
{
using tap::algorithms::SmoothPidConfig;
using tap::can::CanBus;
using tap::motor::DjiMotor;
using tap::motor::MotorId;

constexpr CanBus CHASSIS_CAN_BUS = CanBus::CAN_BUS1;

constexpr MotorId LEFT_FRONT_MOTOR_ID = MotorId::MOTOR2;
constexpr MotorId RIGHT_FRONT_MOTOR_ID = MotorId::MOTOR1;
constexpr MotorId LEFT_BACK_MOTOR_ID = MotorId::MOTOR3;
constexpr MotorId RIGHT_BACK_MOTOR_ID = MotorId::MOTOR4;

constexpr subsystems::chassis::ChassisGeometry CHASSIS_GEOMETRY = {
    .wheelRadius = 0.0762f,  // 6in wheels
    .lengthX = 0.3f,         // 600mm wide chassis
    .lengthY = 0.3f,         // 600mm long chassis
};

/// Wheel angular velocity (rad/s) -> motor output. Shared by all four wheels.
constexpr SmoothPidConfig CHASSIS_WHEEL_PID_CONFIG = {
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = DjiMotor::MAX_OUTPUT_C620,
};

// Target rotation rate while beyblading
constexpr float BEYBLADE_ROTATION_RATE = M_TWOPI;

// Battery voltage estimate of 24V
constexpr float NOMINAL_BATTERY_VOLTAGE_MV = 24000.0f;

// Power limiting
constexpr float STARTING_ENERGY_BUFFER = 60.0f;
constexpr float ENERGY_BUFFER_LIMIT_THRESHOLD = 60.0f;
// Below this value, we clamp to 0 output
constexpr float ENERGY_BUFFER_CRIT_THRESHOLD = 5.0f;

}  // namespace huskybot::standard::constants

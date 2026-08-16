#pragma once

#include <cmath>

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/communication/can/can_bus.hpp"
#include "tap/motor/dji_motor.hpp"

#include "algorithms/controllers/cascade_pid_controller.hpp"
#include "algorithms/controllers/gravity_compensator.hpp"
#include "algorithms/heat/heat_predictor.hpp"
#include "algorithms/transforms/transform_manager.hpp"
#include "control/control_operator_interface.hpp"
#include "subsystems/agitator/agitator_subsystem.hpp"
#include "subsystems/chassis/chassis_kinematics.hpp"
#include "subsystems/turret/turret_subsystem.hpp"

/**
 * Every tuning knob for the standard, in one place. Change values here; the wiring in
 * `standard_control.cpp` shouldn't need to change to tune the robot.
 *
 * Gains that are still 0 have never been run on hardware. Tune each loop from the inside out:
 * velocity loops first (kp until it tracks a step without oscillating, then kd to damp what's
 * left, ki last and small), then the position loop wrapped around it.
 */
namespace huskybot::standard::constants
{
using tap::algorithms::SmoothPidConfig;
using tap::can::CanBus;
using tap::motor::MotorId;

// ---------------------------------------------------------------------------------------------
// Chassis
// ---------------------------------------------------------------------------------------------

constexpr CanBus CHASSIS_CAN_BUS = CanBus::CAN_BUS1;
constexpr MotorId LEFT_FRONT_MOTOR_ID = MotorId::MOTOR2;
constexpr MotorId RIGHT_FRONT_MOTOR_ID = MotorId::MOTOR1;
constexpr MotorId LEFT_BACK_MOTOR_ID = MotorId::MOTOR3;
constexpr MotorId RIGHT_BACK_MOTOR_ID = MotorId::MOTOR4;
/// The right-side motors face the other way, so their encoders count backwards.
constexpr bool LEFT_WHEEL_INVERTED = false;
constexpr bool RIGHT_WHEEL_INVERTED = true;

constexpr subsystems::chassis::ChassisGeometry CHASSIS_GEOMETRY = {
    .wheelRadius = 0.076f,
    .lengthX = 0.175f,
    .lengthY = 0.175f,
};

/// Wheel angular velocity (rad/s) -> motor output. Shared by all four wheels.
constexpr SmoothPidConfig CHASSIS_WHEEL_PID_CONFIG = {
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 16000.0f,
};

/// Counter-clockwise spin rate while beyblading, in rad/s. Higher spins faster but leaves less
/// wheel authority for translation, so the chassis crawls when you also ask it to drive.
constexpr float BEYBLADE_ROTATION_RATE = 4.0f;

/// Battery voltage assumed when estimating chassis power draw, in millivolts.
constexpr float NOMINAL_BATTERY_VOLTAGE_MV = 24000.0f;

/// Power buffer, in joules, that the referee system starts every match with.
constexpr float STARTING_ENERGY_BUFFER = 60.0f;
/// Power limiting starts once the buffer drops below this, in joules. Raise it to start easing
/// off sooner (safer, slower); lower it to run closer to the limit.
constexpr float ENERGY_BUFFER_LIMIT_THRESHOLD = 60.0f;
/// Below this many joules the chassis is cut to zero output, to stay out of the penalty zone.
constexpr float ENERGY_BUFFER_CRIT_THRESHOLD = 5.0f;

// ---------------------------------------------------------------------------------------------
// Turret
// ---------------------------------------------------------------------------------------------

constexpr CanBus TURRET_CAN_BUS = CanBus::CAN_BUS1;
constexpr MotorId YAW_MOTOR_ID = MotorId::MOTOR5;
constexpr MotorId PITCH_MOTOR_ID = MotorId::MOTOR6;
constexpr bool YAW_MOTOR_INVERTED = false;
constexpr bool PITCH_MOTOR_INVERTED = false;

/// Encoder angle (rad) read when the turret points straight forward / level. Measure this by
/// pointing the turret forward and printing `getChassisFrameYaw()` with the offset set to 0.
constexpr subsystems::turret::TurretMotorConfig YAW_MOTOR_CONFIG = {.chassisFrameZeroOffset = 0.0f};
constexpr subsystems::turret::TurretMotorConfig PITCH_MOTOR_CONFIG = {
    .chassisFrameZeroOffset = 0.0f};

/// Angle (rad) -> velocity (rad/s) -> motor output, plus a feedforward straight from the stick.
/// Raise `feedforwardGain` until the turret keeps up with a slew without the position loop
/// having to build up error; the position loop then only cleans up what's left.
constexpr algorithms::controllers::CascadePidControllerConfig YAW_PID_CONFIG = {
    .positionPidConfig = {.kp = 0.0f, .maxOutput = 20.0f},
    .velocityPidConfig = {.kp = 0.0f, .maxOutput = 30000.0f},
    .feedforwardGain = 0.0f,
};

constexpr algorithms::controllers::CascadePidControllerConfig PITCH_PID_CONFIG = {
    .positionPidConfig = {.kp = 0.0f, .maxOutput = 20.0f},
    .velocityPidConfig = {.kp = 0.0f, .maxOutput = 30000.0f},
    .feedforwardGain = 0.0f,
};

/// Where the pitch assembly's center of gravity sits relative to the pitch pivot, and how much
/// output it takes to hold it level. Tune `maxCompensationOutput` by parking the turret level
/// with the position gains at 0 and raising it until the barrel stops sagging.
constexpr algorithms::controllers::GravityCompensatorConfig PITCH_GRAVITY_CONFIG = {
    .cgX = 0.0f,
    .cgZ = 0.0f,
    .maxCompensationOutput = 0.0f,
};

// ---------------------------------------------------------------------------------------------
// Flywheels
// ---------------------------------------------------------------------------------------------

constexpr CanBus FLYWHEEL_CAN_BUS = CanBus::CAN_BUS2;
constexpr MotorId LEFT_FLYWHEEL_MOTOR_ID = MotorId::MOTOR1;
constexpr MotorId RIGHT_FLYWHEEL_MOTOR_ID = MotorId::MOTOR2;
/// The two wheels face each other, so one has to spin backwards to throw the projectile forward.
constexpr bool LEFT_FLYWHEEL_INVERTED = false;
constexpr bool RIGHT_FLYWHEEL_INVERTED = true;

/// Wheel angular velocity (rad/s) -> motor output. Shared by both wheels.
constexpr SmoothPidConfig FLYWHEEL_PID_CONFIG = {
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 16000.0f,
};

/// Flywheel speed (rad/s) to spin up to when firing is armed. This is the overspeed protection:
/// aim under the referee system's limit (22 m/s against a 25 m/s limit) so that barrel wear and
/// projectile variation can't push a shot over it. Measure the muzzle speed the ref system
/// reports and walk this number until it reads ~22 m/s.
constexpr float FLYWHEEL_FIRING_SPEED = 0.0f;

// ---------------------------------------------------------------------------------------------
// Agitator
// ---------------------------------------------------------------------------------------------

constexpr CanBus AGITATOR_CAN_BUS = CanBus::CAN_BUS2;
constexpr MotorId AGITATOR_MOTOR_ID = MotorId::MOTOR7;
constexpr bool AGITATOR_MOTOR_INVERTED = false;

/// One click = one projectile. `setpointIncrement` is 2*pi / (spokes in the feeder wheel), on the
/// output side of the gearbox.
constexpr subsystems::agitator::AgitatorConfig AGITATOR_CONFIG = {
    .setpointIncrement = 2.0f * static_cast<float>(M_PI) / 8.0f,
    .tolerance = 0.05f,
};

/// Position (rad) -> motor output. Wants a stiff kp so a shot completes promptly, and enough kd
/// to keep it from ringing at the setpoint.
constexpr SmoothPidConfig AGITATOR_PID_CONFIG = {
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 16000.0f,
};

/// Heat the referee system charges per projectile. 17mm robots use the 17mm cost.
constexpr float PROJECTILE_HEAT_COST = algorithms::heat::HeatPredictor::HEAT_COST_17MM;

// ---------------------------------------------------------------------------------------------
// Operator input
// ---------------------------------------------------------------------------------------------

/// How the DR16's sticks, wheel, keyboard and mouse map onto robot motion. Negate any sensitivity
/// to invert that axis; which way a stick has to move depends on how the remote is held and the
/// motors are mounted, so expect to flip a sign or two the first time this runs.
constexpr control::ControlOperatorInterfaceConfig CONTROL_OPERATOR_INTERFACE_CONFIG = {
    .maxTranslationSpeed = 3.0f,
    .maxRotationSpeed = 4.0f,
    .maxTurretYawSpeed = 6.0f,
    .maxTurretPitchSpeed = 3.0f,
    // Mouse counts run to roughly +/-100 on a fast flick.
    .mouseYawSensitivity = -0.06f,
    .mousePitchSensitivity = -0.03f,
    // The DR16's sticks don't quite recenter; anything under this is treated as centered.
    .stickDeadzone = 0.03f,
};

}  // namespace huskybot::standard::constants

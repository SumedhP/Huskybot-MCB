#if defined(TARGET_STANDARD) && !defined(ENV_UNIT_TESTS)

#include "tap/control/governor/governor_limited_command.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/remote_map_state.hpp"
#include "tap/control/toggle_command_mapping.hpp"
#include "tap/motor/dji_motor.hpp"

#include "algorithms/controllers/cascade_pid_controller.hpp"
#include "algorithms/controllers/gravity_compensator.hpp"
#include "algorithms/heat/heat_predictor.hpp"
#include "algorithms/transforms/transform_manager.hpp"
#include "communication/chassis_power_sensors.hpp"
#include "constants/standard_constants.hpp"
#include "control/control_operator_interface.hpp"
#include "control/governor/flywheels_on_governor.hpp"
#include "control/governor/heat_limit_governor.hpp"
#include "robot/robot_control.hpp"
#include "subsystems/agitator/agitator_fire_command.hpp"
#include "subsystems/agitator/agitator_subsystem.hpp"
#include "subsystems/chassis/chassis_beyblade_command.hpp"
#include "subsystems/chassis/chassis_drive_command.hpp"
#include "subsystems/chassis/chassis_kinematics.hpp"
#include "subsystems/chassis/chassis_subsystem.hpp"
#include "subsystems/flywheel/flywheel_subsystem.hpp"
#include "subsystems/flywheel/flywheels_on_command.hpp"
#include "subsystems/turret/imu_calibrate_command.hpp"
#include "subsystems/turret/turret_control_command.hpp"
#include "subsystems/turret/turret_subsystem.hpp"
#include "util/remote_safe_disconnect.hpp"

#include "drivers_singleton.hpp"

using namespace tap::communication::serial;
using namespace tap::motor;
// Pulled in one name at a time: `using namespace tap::control` would make plain `chassis` mean
// both `tap::control::chassis` and `huskybot::subsystems::chassis`.
using tap::control::Command;
using tap::control::HoldCommandMapping;
using tap::control::HoldRepeatCommandMapping;
using tap::control::RemoteMapState;
using tap::control::ToggleCommandMapping;
using tap::control::governor::GovernorLimitedCommand;
using namespace huskybot::algorithms;
using namespace huskybot::control::governor;
using namespace huskybot::standard::constants;
using namespace huskybot::subsystems;

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here because this file defines all
 *      subsystems and commands, and so must pass the single statically allocated Drivers
 *      instance to all of them.
 */
huskybot::driversFunc drivers = huskybot::DoNotUse_getDrivers;

namespace standard_control
{
/* motors -------------------------------------------------------------------*/
DjiMotor leftFrontChassisMotor(
    drivers(),
    LEFT_FRONT_MOTOR_ID,
    CHASSIS_CAN_BUS,
    false,
    "Left Front Chassis",
    false,
    DjiMotorEncoder::GEAR_RATIO_M3508);

DjiMotor rightFrontChassisMotor(
    drivers(),
    RIGHT_FRONT_MOTOR_ID,
    CHASSIS_CAN_BUS,
    true,
    "Right Front Chassis",
    false,
    DjiMotorEncoder::GEAR_RATIO_M3508);

DjiMotor leftBackChassisMotor(
    drivers(),
    LEFT_BACK_MOTOR_ID,
    CHASSIS_CAN_BUS,
    false,
    "Left Back Chassis",
    false,
    DjiMotorEncoder::GEAR_RATIO_M3508);

DjiMotor rightBackChassisMotor(
    drivers(),
    RIGHT_BACK_MOTOR_ID,
    CHASSIS_CAN_BUS,
    true,
    "Right Back Chassis",
    false,
    DjiMotorEncoder::GEAR_RATIO_M3508);

DjiMotor yawMotor(
    drivers(),
    YAW_MOTOR_ID,
    TURRET_CAN_BUS,
    false,
    "Turret Yaw",
    false,
    DjiMotorEncoder::GEAR_RATIO_GM6020);

DjiMotor pitchMotor(
    drivers(),
    PITCH_MOTOR_ID,
    TURRET_CAN_BUS,
    false,
    "Turret Pitch",
    false,
    DjiMotorEncoder::GEAR_RATIO_GM6020);

DjiMotor leftFlywheelMotor(
    drivers(),
    LEFT_FLYWHEEL_MOTOR_ID,
    FLYWHEEL_CAN_BUS,
    false,
    "Left Flywheel",
    false,
    DjiMotorEncoder::GEAR_RATIO_M3508);

DjiMotor rightFlywheelMotor(
    drivers(),
    RIGHT_FLYWHEEL_MOTOR_ID,
    FLYWHEEL_CAN_BUS,
    true,
    "Right Flywheel",
    false,
    DjiMotorEncoder::GEAR_RATIO_M3508);

DjiMotor agitatorMotor(
    drivers(),
    AGITATOR_MOTOR_ID,
    AGITATOR_CAN_BUS,
    false,
    "Agitator",
    false,
    DjiMotorEncoder::GEAR_RATIO_M2006);

/* power limiting -----------------------------------------------------------*/
huskybot::communication::DummyVoltageSensor voltageSensor(NOMINAL_BATTERY_VOLTAGE_MV);
huskybot::communication::DummyCurrentSensor currentSensor;

tap::control::chassis::PowerLimiter powerLimiter(
    drivers(),
    &currentSensor,
    &voltageSensor,
    STARTING_ENERGY_BUFFER,
    ENERGY_BUFFER_LIMIT_THRESHOLD,
    ENERGY_BUFFER_CRIT_THRESHOLD);

/* subsystems ---------------------------------------------------------------*/
/// Swap `omniWheelMatrix` for `mecanumWheelMatrix` to run mecanum wheels on the same geometry.
chassis::ChassisSubsystem chassis(
    drivers(),
    leftFrontChassisMotor,
    rightFrontChassisMotor,
    leftBackChassisMotor,
    rightBackChassisMotor,
    chassis::omniWheelMatrix(CHASSIS_GEOMETRY),
    CHASSIS_WHEEL_PID_CONFIG,
    powerLimiter);

turret::TurretSubsystem turret(
    drivers(),
    yawMotor,
    pitchMotor,
    drivers()->bmi088,
    YAW_MOTOR_CONFIG,
    PITCH_MOTOR_CONFIG);

flywheel::FlywheelSubsystem flywheels(
    drivers(),
    leftFlywheelMotor,
    rightFlywheelMotor,
    FLYWHEEL_PID_CONFIG);

agitator::AgitatorSubsystem agitator(
    drivers(),
    agitatorMotor,
    AGITATOR_CONFIG,
    AGITATOR_PID_CONFIG);

/* operator input -----------------------------------------------------------*/
huskybot::control::ControlOperatorInterface controlOperatorInterface(
    *drivers(),
    CONTROL_OPERATOR_INTERFACE_CONFIG);

/* algorithms ---------------------------------------------------------------*/
transforms::TransformManager transformManager(turret);

heat::HeatPredictor heatPredictor(*drivers(), PROJECTILE_HEAT_COST);

controllers::CascadePidController yawController(YAW_PID_CONFIG);
controllers::CascadePidController pitchController(PITCH_PID_CONFIG);
controllers::GravityCompensator pitchGravityCompensator(PITCH_GRAVITY_CONFIG);

/// Calibration drives the turret off the encoders instead of the IMU, so it gets its own pair of
/// controllers rather than borrowing the world-frame ones.
controllers::CascadePidController imuCalibrateYawController(IMU_CALIBRATE_YAW_PID_CONFIG);
controllers::CascadePidController imuCalibratePitchController(IMU_CALIBRATE_PITCH_PID_CONFIG);

/* commands -----------------------------------------------------------------*/
chassis::ChassisDriveCommand chassisDriveCommand(
    chassis,
    controlOperatorInterface,
    transformManager);

chassis::ChassisBeybladeCommand chassisBeybladeCommand(
    chassis,
    controlOperatorInterface,
    transformManager,
    BEYBLADE_ROTATION_RATE);

turret::TurretControlCommand turretControlCommand(
    turret,
    controlOperatorInterface,
    yawController,
    pitchController,
    pitchGravityCompensator);

turret::ImuCalibrateCommand imuCalibrateCommand(
    turret,
    drivers()->bmi088,
    imuCalibrateYawController,
    imuCalibratePitchController,
    IMU_CALIBRATE_CONFIG);

flywheel::FlywheelsOnCommand spinFlywheels(flywheels, FLYWHEEL_FIRING_SPEED);
flywheel::FlywheelsOnCommand stopFlywheels(flywheels, 0.0f);

agitator::AgitatorFireCommand agitatorFireCommand(agitator, heatPredictor);

/* governors ----------------------------------------------------------------*/
HeatLimitGovernor heatLimitGovernor(heatPredictor);
FlywheelsOnGovernor flywheelsOnGovernor(flywheels);

GovernorLimitedCommand<2> governedFireCommand(
    {&agitator},
    agitatorFireCommand,
    {&heatLimitGovernor, &flywheelsOnGovernor});

huskybot::util::RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

/* remote mappings ----------------------------------------------------------*/
RemoteMapState leftSwitchDown(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::DOWN);
RemoteMapState leftSwitchUp(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP);
RemoteMapState rightSwitchMid(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::MID);
RemoteMapState rightSwitchUp(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP);
RemoteMapState leftMouseButtonPressed(RemoteMapState::MouseButton::LEFT);
RemoteMapState fPressed({Remote::Key::F});

/* setup --------------------------------------------------------------------*/
void initializeSubsystems()
{
    chassis.initialize();
    turret.initialize();
    flywheels.initialize();
    agitator.initialize();
}

void registerStandardSubsystems(tap::Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&chassis);
    drivers->commandScheduler.registerSubsystem(&turret);
    drivers->commandScheduler.registerSubsystem(&flywheels);
    drivers->commandScheduler.registerSubsystem(&agitator);
}

void setDefaultStandardCommands(tap::Drivers *)
{
    chassis.setDefaultCommand(&chassisDriveCommand);
    turret.setDefaultCommand(&turretControlCommand);
    flywheels.setDefaultCommand(&stopFlywheels);
}

void registerStandardIoMappings(tap::Drivers *drivers)
{
    // Left switch up: recalibrate the turret IMU.
    drivers->commandMapper.addMap(std::make_unique<HoldCommandMapping>(
        drivers,
        std::vector<Command *>{&imuCalibrateCommand},
        &leftSwitchUp));

    // Left switch down: beyblade instead of the default straight drive.
    drivers->commandMapper.addMap(std::make_unique<HoldCommandMapping>(
        drivers,
        std::vector<Command *>{&chassisBeybladeCommand},
        &leftSwitchDown));

    // Right switch mid: spin the flywheels up and hold them there, ready to fire.
    drivers->commandMapper.addMap(std::make_unique<HoldCommandMapping>(
        drivers,
        std::vector<Command *>{&spinFlywheels},
        &rightSwitchMid));

    // Right switch up: keep the flywheels spinning and fire for as long as it's held, as fast as
    // the heat limit allows.
    drivers->commandMapper.addMap(std::make_unique<HoldRepeatCommandMapping>(
        drivers,
        std::vector<Command *>{&spinFlywheels, &governedFireCommand},
        &rightSwitchUp,
        true));

    // Left mouse button: same fire behavior as right switch up.
    drivers->commandMapper.addMap(std::make_unique<HoldRepeatCommandMapping>(
        drivers,
        std::vector<Command *>{&spinFlywheels, &governedFireCommand},
        &leftMouseButtonPressed,
        true));

    // F: toggle beyblade on and off.
    drivers->commandMapper.addMap(std::make_unique<ToggleCommandMapping>(
        drivers,
        std::vector<Command *>{&chassisBeybladeCommand},
        &fPressed));
}
}  // namespace standard_control

namespace huskybot
{
void initSubsystemCommands(tap::Drivers &drivers)
{
    drivers.commandScheduler.setSafeDisconnectFunction(
        &standard_control::remoteSafeDisconnectFunction);
    standard_control::initializeSubsystems();
    standard_control::registerStandardSubsystems(&drivers);
    standard_control::setDefaultStandardCommands(&drivers);
    standard_control::registerStandardIoMappings(&drivers);

    // The IMU drifts until it has been calibrated once, so do it on boot rather than waiting for
    // the operator to ask.
    drivers.commandScheduler.addCommand(&standard_control::imuCalibrateCommand);
}

void updateRobotIo(tap::Drivers &)
{
    // If you had a custom power sensor or IMU, update it here.
}

void updateRobotState(tap::Drivers &)
{
    standard_control::heatPredictor.updateHeatCost();
    standard_control::transformManager.update();
}
}  // namespace huskybot

#endif

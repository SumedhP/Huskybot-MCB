#if defined(TARGET_STANDARD) && !defined(ENV_UNIT_TESTS)

#include "tap/control/governor/governor_limited_command.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/remote_map_state.hpp"
#include "tap/motor/dji_motor.hpp"

#include "algorithms/controllers/cascade_pid_controller.hpp"
#include "algorithms/controllers/gravity_compensator.hpp"
#include "algorithms/transforms/transform_manager.hpp"
#include "communication/chassis_power_sensors.hpp"
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
#include "subsystems/turret/turret_control_command.hpp"
#include "subsystems/turret/turret_subsystem.hpp"

#include "drivers_singleton.hpp"
#include "standard_constants.hpp"

using namespace tap::communication::serial;
using namespace tap::motor;
// Pulled in one name at a time: `using namespace tap::control` would make plain `chassis` mean
// both `tap::control::chassis` and `huskybot::subsystems::chassis`.
using tap::control::Command;
using tap::control::HoldCommandMapping;
using tap::control::HoldRepeatCommandMapping;
using tap::control::RemoteMapState;
using tap::control::SafeDisconnectFunction;
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
huskybot::standard::driversFunc drivers = huskybot::standard::DoNotUse_getDrivers;

namespace standard_control
{
/* motors -------------------------------------------------------------------*/
DjiMotor leftFrontChassisMotor(
    drivers(),
    LEFT_FRONT_MOTOR_ID,
    CHASSIS_CAN_BUS,
    LEFT_WHEEL_INVERTED,
    "Left Front Chassis",
    false,
    DjiMotorEncoder::GEAR_RATIO_M3508);

DjiMotor rightFrontChassisMotor(
    drivers(),
    RIGHT_FRONT_MOTOR_ID,
    CHASSIS_CAN_BUS,
    RIGHT_WHEEL_INVERTED,
    "Right Front Chassis",
    false,
    DjiMotorEncoder::GEAR_RATIO_M3508);

DjiMotor leftBackChassisMotor(
    drivers(),
    LEFT_BACK_MOTOR_ID,
    CHASSIS_CAN_BUS,
    LEFT_WHEEL_INVERTED,
    "Left Back Chassis",
    false,
    DjiMotorEncoder::GEAR_RATIO_M3508);

DjiMotor rightBackChassisMotor(
    drivers(),
    RIGHT_BACK_MOTOR_ID,
    CHASSIS_CAN_BUS,
    RIGHT_WHEEL_INVERTED,
    "Right Back Chassis",
    false,
    DjiMotorEncoder::GEAR_RATIO_M3508);

DjiMotor yawMotor(
    drivers(),
    YAW_MOTOR_ID,
    TURRET_CAN_BUS,
    YAW_MOTOR_INVERTED,
    "Turret Yaw",
    false,
    DjiMotorEncoder::GEAR_RATIO_GM6020);

DjiMotor pitchMotor(
    drivers(),
    PITCH_MOTOR_ID,
    TURRET_CAN_BUS,
    PITCH_MOTOR_INVERTED,
    "Turret Pitch",
    false,
    DjiMotorEncoder::GEAR_RATIO_GM6020);

DjiMotor leftFlywheelMotor(
    drivers(),
    LEFT_FLYWHEEL_MOTOR_ID,
    FLYWHEEL_CAN_BUS,
    LEFT_FLYWHEEL_INVERTED,
    "Left Flywheel",
    false,
    DjiMotorEncoder::GEAR_RATIO_M3508);

DjiMotor rightFlywheelMotor(
    drivers(),
    RIGHT_FLYWHEEL_MOTOR_ID,
    FLYWHEEL_CAN_BUS,
    RIGHT_FLYWHEEL_INVERTED,
    "Right Flywheel",
    false,
    DjiMotorEncoder::GEAR_RATIO_M3508);

DjiMotor agitatorMotor(
    drivers(),
    AGITATOR_MOTOR_ID,
    AGITATOR_CAN_BUS,
    AGITATOR_MOTOR_INVERTED,
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

/* algorithms ---------------------------------------------------------------*/
transforms::TransformManager transformManager(turret, TRANSFORM_CONFIG);

controllers::CascadePidController yawController(YAW_PID_CONFIG);
controllers::CascadePidController pitchController(PITCH_PID_CONFIG);
controllers::GravityCompensator pitchGravityCompensator(PITCH_GRAVITY_CONFIG);

/* commands -----------------------------------------------------------------*/
chassis::ChassisDriveCommand chassisDriveCommand(
    chassis,
    drivers()->controlOperatorInterface,
    transformManager);

chassis::ChassisBeybladeCommand chassisBeybladeCommand(
    chassis,
    drivers()->controlOperatorInterface,
    transformManager,
    BEYBLADE_ROTATION_RATE);

turret::TurretControlCommand turretControlCommand(
    turret,
    drivers()->controlOperatorInterface,
    yawController,
    pitchController,
    pitchGravityCompensator);

flywheel::FlywheelsOnCommand spinFlywheels(flywheels, FLYWHEEL_FIRING_SPEED);
flywheel::FlywheelsOnCommand stopFlywheels(flywheels, 0.0f);

agitator::AgitatorFireCommand agitatorFireCommand(agitator, drivers()->heatPredictor);

/* governors ----------------------------------------------------------------*/
HeatLimitGovernor heatLimitGovernor(drivers()->heatPredictor);
FlywheelsOnGovernor flywheelsOnGovernor(flywheels);

/// Firing is only allowed when the referee system says we have heat left and when the flywheels
/// are actually up to speed -- a shot fired into stationary flywheels jams the barrel.
GovernorLimitedCommand<2> governedFireCommand(
    {&agitator},
    agitatorFireCommand,
    {&heatLimitGovernor, &flywheelsOnGovernor});

/* safe disconnect ----------------------------------------------------------*/
/// Everything stops when the remote is unplugged or turned off.
class RemoteSafeDisconnectFunction : public SafeDisconnectFunction
{
public:
    explicit RemoteSafeDisconnectFunction(huskybot::standard::Drivers *drivers) : drivers(drivers)
    {
    }

    bool operator()() override { return !drivers->remote.isConnected(); }

private:
    huskybot::standard::Drivers *drivers;
};

RemoteSafeDisconnectFunction remoteSafeDisconnectFunction(drivers());

/* remote mappings ----------------------------------------------------------*/
// The mappings hold these by pointer, so they have to outlive the mappings themselves.
RemoteMapState leftSwitchUpState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP);
RemoteMapState rightSwitchMidState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::MID);
RemoteMapState rightSwitchUpState(Remote::Switch::RIGHT_SWITCH, Remote::SwitchState::UP);

/* setup --------------------------------------------------------------------*/
void initializeSubsystems()
{
    chassis.initialize();
    turret.initialize();
    flywheels.initialize();
    agitator.initialize();
}

void registerStandardSubsystems(huskybot::standard::Drivers *drivers)
{
    drivers->commandScheduler.registerSubsystem(&chassis);
    drivers->commandScheduler.registerSubsystem(&turret);
    drivers->commandScheduler.registerSubsystem(&flywheels);
    drivers->commandScheduler.registerSubsystem(&agitator);
}

void setDefaultStandardCommands(huskybot::standard::Drivers *)
{
    chassis.setDefaultCommand(&chassisDriveCommand);
    turret.setDefaultCommand(&turretControlCommand);
    flywheels.setDefaultCommand(&stopFlywheels);
}

void registerStandardIoMappings(huskybot::standard::Drivers *drivers)
{
    // Left switch up: beyblade instead of the default straight drive.
    drivers->commandMapper.addMap(std::make_unique<HoldCommandMapping>(
        drivers,
        std::vector<Command *>{&chassisBeybladeCommand},
        &leftSwitchUpState));

    // Right switch mid: spin the flywheels up and hold them there, ready to fire.
    drivers->commandMapper.addMap(std::make_unique<HoldCommandMapping>(
        drivers,
        std::vector<Command *>{&spinFlywheels},
        &rightSwitchMidState));

    // Right switch up: keep the flywheels spinning and fire for as long as it's held, as fast as
    // the heat limit allows.
    drivers->commandMapper.addMap(std::make_unique<HoldRepeatCommandMapping>(
        drivers,
        std::vector<Command *>{&spinFlywheels, &governedFireCommand},
        &rightSwitchUpState,
        true));
}
}  // namespace standard_control

namespace huskybot::standard
{
void initSubsystemCommands(Drivers *drivers)
{
    drivers->commandScheduler.setSafeDisconnectFunction(
        &standard_control::remoteSafeDisconnectFunction);
    standard_control::initializeSubsystems();
    standard_control::registerStandardSubsystems(drivers);
    standard_control::setDefaultStandardCommands(drivers);
    standard_control::registerStandardIoMappings(drivers);
}

void updateRobotIo(Drivers *drivers)
{
    drivers->heatPredictor.updateHeatCost();
    standard_control::transformManager.update();
}
}  // namespace huskybot::standard

#endif

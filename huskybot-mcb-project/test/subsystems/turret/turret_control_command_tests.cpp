#include <cmath>

#include <gtest/gtest.h>

#include "tap/architecture/clock.hpp"
#include "tap/drivers.hpp"
#include "tap/mock/abstract_imu_mock.hpp"
#include "tap/mock/motor_interface_mock.hpp"

#include "subsystems/turret/turret_control_command.hpp"
#include "subsystems/turret/turret_subsystem.hpp"

using namespace testing;
using namespace huskybot::subsystems::turret;
using huskybot::algorithms::controllers::CascadePidController;
using huskybot::algorithms::controllers::GravityCompensator;

namespace
{
class StubOperatorInterface : public huskybot::control::ControlOperatorInterface
{
public:
    float getTurretYawInput() override { return yawInput; }
    float getTurretPitchInput() override { return pitchInput; }

    float yawInput = 0.0f;
    float pitchInput = 0.0f;
};
}  // namespace

class TurretControlCommandTest : public Test
{
protected:
    TurretControlCommandTest()
        : turret(
              &drivers,
              yawMotor,
              pitchMotor,
              imu,
              {.chassisFrameZeroOffset = 0.0f},
              {.chassisFrameZeroOffset = 0.0f}),
          yawController(
              {.positionPidConfig = {.kp = 1000.0f, .maxOutput = 20000.0f},
               .velocityPidConfig = {.kp = 1000.0f, .maxOutput = 20000.0f},
               .feedforwardGain = 1.0f}),
          pitchController(
              {.positionPidConfig = {.kp = 1000.0f, .maxOutput = 20000.0f},
               .velocityPidConfig = {.kp = 1000.0f, .maxOutput = 20000.0f},
               .feedforwardGain = 1.0f}),
          pitchGravityCompensator({.cgX = 0.0f, .cgZ = 0.0f, .maxCompensationOutput = 0.0f}),
          command(
              turret,
              operatorInterface,
              yawController,
              pitchController,
              pitchGravityCompensator)
    {
    }

    void SetUp() override
    {
        ON_CALL(yawMotor, isMotorOnline).WillByDefault(Return(true));
        ON_CALL(*yawMotor.getEncoder(), getPosition)
            .WillByDefault(Return(tap::algorithms::WrappedFloat(0, -M_PI, M_PI)));
        ON_CALL(*yawMotor.getEncoder(), getVelocity).WillByDefault(Return(0.0f));

        ON_CALL(pitchMotor, isMotorOnline).WillByDefault(Return(true));
        ON_CALL(*pitchMotor.getEncoder(), getPosition)
            .WillByDefault(Return(tap::algorithms::WrappedFloat(0, -M_PI, M_PI)));
        ON_CALL(*pitchMotor.getEncoder(), getVelocity).WillByDefault(Return(0.0f));

        ON_CALL(imu, getYaw).WillByDefault(ReturnPointee(&imuYaw));
        ON_CALL(imu, getPitch).WillByDefault(Return(0.0f));
        ON_CALL(imu, getGz).WillByDefault(Return(0.0f));
        ON_CALL(imu, getGy).WillByDefault(Return(0.0f));
    }

    tap::arch::clock::ClockStub clock;
    tap::Drivers drivers;
    NiceMock<tap::mock::MotorInterfaceMock> yawMotor;
    NiceMock<tap::mock::MotorInterfaceMock> pitchMotor;
    NiceMock<tap::mock::AbstractImuMock> imu;
    TurretSubsystem turret;
    CascadePidController yawController;
    CascadePidController pitchController;
    GravityCompensator pitchGravityCompensator;
    StubOperatorInterface operatorInterface;
    TurretControlCommand command;

    float imuYaw = 0.0f;
};

TEST_F(TurretControlCommandTest, isReady_true_when_turret_online)
{
    EXPECT_TRUE(command.isReady());
}

TEST_F(TurretControlCommandTest, isReady_false_when_turret_offline)
{
    ON_CALL(yawMotor, isMotorOnline).WillByDefault(Return(false));

    EXPECT_FALSE(command.isReady());
}

TEST_F(TurretControlCommandTest, isFinished_always_false) { EXPECT_FALSE(command.isFinished()); }

TEST_F(TurretControlCommandTest, execute_holds_at_initial_setpoint_with_no_operator_input)
{
    command.initialize();

    clock.time += 10;

    // Setpoint == measured angle and no feedforward, so the cascade should command ~0 output.
    EXPECT_CALL(yawMotor, setDesiredOutput(0));
    EXPECT_CALL(pitchMotor, setDesiredOutput(0));

    command.execute();
}

TEST_F(TurretControlCommandTest, execute_commands_output_when_operator_input_present)
{
    command.initialize();
    operatorInterface.yawInput = 1.0f;

    clock.time += 10;

    EXPECT_CALL(yawMotor, setDesiredOutput(Gt(0)));

    command.execute();
}

TEST_F(TurretControlCommandTest, execute_adds_gravity_compensation_to_pitch_output)
{
    pitchGravityCompensator =
        GravityCompensator({.cgX = 1.0f, .cgZ = 0.0f, .maxCompensationOutput = 5000.0f});

    command.initialize();
    clock.time += 10;

    // Setpoint == measured angle and no operator input, so absent gravity comp this would be 0.
    EXPECT_CALL(pitchMotor, setDesiredOutput(5000));

    command.execute();
}

TEST_F(TurretControlCommandTest, end_zeroes_both_motor_outputs)
{
    command.initialize();

    EXPECT_CALL(yawMotor, setDesiredOutput(0));
    EXPECT_CALL(pitchMotor, setDesiredOutput(0));

    command.end(false);
}

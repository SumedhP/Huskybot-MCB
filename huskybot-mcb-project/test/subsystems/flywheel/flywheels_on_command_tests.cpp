#include <gtest/gtest.h>

#include "tap/drivers.hpp"
#include "tap/mock/motor_interface_mock.hpp"

#include "subsystems/flywheel/flywheel_subsystem.hpp"
#include "subsystems/flywheel/flywheels_on_command.hpp"

using namespace testing;
using namespace huskybot::subsystems::flywheel;

class FlywheelsOnCommandTest : public Test
{
protected:
    FlywheelsOnCommandTest()
        : flywheel(&drivers, leftMotor, rightMotor, {.kp = 10000.0f, .maxOutput = 16000}),
          command(flywheel, 5000.0f)
    {
    }

    void SetUp() override
    {
        ON_CALL(leftMotor, isMotorOnline).WillByDefault(Return(true));
        ON_CALL(rightMotor, isMotorOnline).WillByDefault(Return(true));
    }

    tap::Drivers drivers;
    NiceMock<tap::mock::MotorInterfaceMock> leftMotor;
    NiceMock<tap::mock::MotorInterfaceMock> rightMotor;
    FlywheelSubsystem flywheel;
    FlywheelsOnCommand command;
};

TEST_F(FlywheelsOnCommandTest, initialize_sets_desired_speed)
{
    command.initialize();

    EXPECT_FLOAT_EQ(5000.0f, flywheel.getDesiredSpeed());
}

TEST_F(FlywheelsOnCommandTest, end_zeroes_desired_speed)
{
    command.initialize();
    command.end(false);

    EXPECT_FLOAT_EQ(0.0f, flywheel.getDesiredSpeed());
}

TEST_F(FlywheelsOnCommandTest, isFinished_always_false) { EXPECT_FALSE(command.isFinished()); }

TEST_F(FlywheelsOnCommandTest, isReady_true_when_flywheel_online)
{
    EXPECT_TRUE(command.isReady());
}

TEST_F(FlywheelsOnCommandTest, isReady_false_when_flywheel_offline)
{
    ON_CALL(leftMotor, isMotorOnline).WillByDefault(Return(false));

    EXPECT_FALSE(command.isReady());
}

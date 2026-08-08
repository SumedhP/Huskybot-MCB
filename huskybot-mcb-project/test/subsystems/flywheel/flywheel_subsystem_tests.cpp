#include <gtest/gtest.h>

#include "tap/architecture/clock.hpp"
#include "tap/drivers.hpp"
#include "tap/mock/motor_interface_mock.hpp"

#include "subsystems/flywheel/flywheel_subsystem.hpp"

using namespace testing;
using namespace huskybot::subsystems::flywheel;

class FlywheelSubsystemTest : public Test
{
protected:
    FlywheelSubsystemTest()
        : flywheel(&drivers, leftMotor, rightMotor, {.kp = 10000.0f, .maxOutput = 16000})
    {
    }

    void SetUp() override
    {
        ON_CALL(leftMotor, isMotorOnline).WillByDefault(ReturnPointee(&leftMotorOnline));
        ON_CALL(*leftMotor.getEncoder(), getVelocity).WillByDefault(ReturnPointee(&leftVelocity));

        ON_CALL(rightMotor, isMotorOnline).WillByDefault(ReturnPointee(&rightMotorOnline));
        ON_CALL(*rightMotor.getEncoder(), getVelocity).WillByDefault(ReturnPointee(&rightVelocity));
    }

    tap::arch::clock::ClockStub clock;
    tap::Drivers drivers;
    NiceMock<tap::mock::MotorInterfaceMock> leftMotor;
    NiceMock<tap::mock::MotorInterfaceMock> rightMotor;
    FlywheelSubsystem flywheel;

    bool leftMotorOnline = true;
    bool rightMotorOnline = true;
    float leftVelocity = 0.0f;
    float rightVelocity = 0.0f;
};

TEST_F(FlywheelSubsystemTest, initialize_initializes_both_motors)
{
    EXPECT_CALL(leftMotor, initialize);
    EXPECT_CALL(rightMotor, initialize);

    flywheel.initialize();
}

TEST_F(FlywheelSubsystemTest, isOnline_true_when_both_motors_online)
{
    EXPECT_TRUE(flywheel.isOnline());
}

TEST_F(FlywheelSubsystemTest, isOnline_false_when_either_motor_offline)
{
    leftMotorOnline = false;

    EXPECT_FALSE(flywheel.isOnline());

    leftMotorOnline = true;
    rightMotorOnline = false;

    EXPECT_FALSE(flywheel.isOnline());
}

TEST_F(FlywheelSubsystemTest, getDesiredSpeed_reflects_setDesiredSpeed)
{
    flywheel.setDesiredSpeed(5000.0f);

    EXPECT_FLOAT_EQ(5000.0f, flywheel.getDesiredSpeed());
}

TEST_F(FlywheelSubsystemTest, getCurrentSpeed_returns_slower_of_the_two_motors)
{
    leftVelocity = 4000.0f;
    rightVelocity = 3000.0f;

    EXPECT_FLOAT_EQ(3000.0f, flywheel.getCurrentSpeed());

    leftVelocity = 1000.0f;
    rightVelocity = 5000.0f;

    EXPECT_FLOAT_EQ(1000.0f, flywheel.getCurrentSpeed());
}

TEST_F(FlywheelSubsystemTest, refresh_commands_both_motors_towards_setpoint)
{
    flywheel.setDesiredSpeed(5000.0f);

    clock.time += 10;

    EXPECT_CALL(leftMotor, setDesiredOutput(Gt(0)));
    EXPECT_CALL(rightMotor, setDesiredOutput(Gt(0)));

    flywheel.refresh();
}

TEST_F(FlywheelSubsystemTest, refreshSafeDisconnect_zeroes_both_motor_outputs)
{
    flywheel.setDesiredSpeed(5000.0f);

    EXPECT_CALL(leftMotor, setDesiredOutput(0));
    EXPECT_CALL(rightMotor, setDesiredOutput(0));

    flywheel.refreshSafeDisconnect();
}

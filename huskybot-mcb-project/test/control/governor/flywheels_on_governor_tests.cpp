#include <gtest/gtest.h>

#include "tap/architecture/clock.hpp"
#include "tap/drivers.hpp"
#include "tap/mock/motor_interface_mock.hpp"

#include "control/governor/flywheels_on_governor.hpp"
#include "subsystems/flywheel/flywheel_subsystem.hpp"

using namespace testing;
using namespace huskybot::subsystems::flywheel;
using namespace huskybot::control::governor;

class FlywheelsOnGovernorTest : public Test
{
protected:
    FlywheelsOnGovernorTest()
        : flywheel(&drivers, leftMotor, rightMotor, {.kp = 1.0f, .maxOutput = 16000}),
          governor(flywheel)
    {
    }

    void SetUp() override
    {
        ON_CALL(*leftMotor.getEncoder(), getVelocity).WillByDefault(ReturnPointee(&leftVelocity));
        ON_CALL(*rightMotor.getEncoder(), getVelocity)
            .WillByDefault(ReturnPointee(&rightVelocity));
    }

    tap::arch::clock::ClockStub clock;
    tap::Drivers drivers;
    NiceMock<tap::mock::MotorInterfaceMock> leftMotor;
    NiceMock<tap::mock::MotorInterfaceMock> rightMotor;
    FlywheelSubsystem flywheel;
    FlywheelsOnGovernor governor;

    float leftVelocity = 0.0f;
    float rightVelocity = 0.0f;
};

TEST_F(FlywheelsOnGovernorTest, isReady_false_when_both_motors_stopped)
{
    EXPECT_FALSE(governor.isReady());
}

TEST_F(FlywheelsOnGovernorTest, isReady_true_once_both_motors_above_threshold)
{
    leftVelocity = 5000.0f;
    rightVelocity = 5000.0f;

    EXPECT_TRUE(governor.isReady());
}

TEST_F(FlywheelsOnGovernorTest, isReady_false_when_only_one_motor_above_threshold)
{
    leftVelocity = 5000.0f;
    rightVelocity = 0.0f;

    EXPECT_FALSE(governor.isReady());
}

TEST_F(FlywheelsOnGovernorTest, isReady_false_again_once_speed_drops_back_to_threshold)
{
    leftVelocity = 5000.0f;
    rightVelocity = 5000.0f;

    EXPECT_TRUE(governor.isReady());

    leftVelocity = 0.0f;
    rightVelocity = 0.0f;

    EXPECT_FALSE(governor.isReady());
}

TEST_F(FlywheelsOnGovernorTest, isFinished_true_when_both_motors_stopped)
{
    EXPECT_TRUE(governor.isFinished());
}

TEST_F(FlywheelsOnGovernorTest, isFinished_false_once_both_motors_above_threshold)
{
    leftVelocity = 5000.0f;
    rightVelocity = 5000.0f;

    EXPECT_FALSE(governor.isFinished());
}

#include <cmath>

#include <gtest/gtest.h>

#include "tap/architecture/clock.hpp"
#include "tap/drivers.hpp"
#include "tap/mock/motor_interface_mock.hpp"

#include "subsystems/agitator/agitator_subsystem.hpp"

using namespace testing;
using namespace huskybot::subsystems::agitator;

class AgitatorSubsystemTest : public Test
{
protected:
    AgitatorSubsystemTest()
        : agitator(
              &drivers,
              motor,
              {.setpointIncrement = M_PI / 4, .tolerance = 0.01f},
              {.kp = 10000.0f, .maxOutput = 16000})
    {
    }

    void SetUp() override
    {
        ON_CALL(motor, isMotorOnline).WillByDefault(ReturnPointee(&motorOnline));
        ON_CALL(*motor.getEncoder(), getPosition).WillByDefault(ReturnPointee(&position));
        ON_CALL(*motor.getEncoder(), getVelocity).WillByDefault(Return(0.0f));
    }

    tap::arch::clock::ClockStub clock;
    tap::Drivers drivers;
    NiceMock<tap::mock::MotorInterfaceMock> motor;
    AgitatorSubsystem agitator;

    tap::algorithms::WrappedFloat position = tap::algorithms::WrappedFloat(0, -M_PI, M_PI);
    bool motorOnline = true;
};

TEST_F(AgitatorSubsystemTest, initialize_initializes_motor)
{
    EXPECT_CALL(motor, initialize);

    agitator.initialize();
}

TEST_F(AgitatorSubsystemTest, isOnline_reflects_motor_online_state)
{
    EXPECT_TRUE(agitator.isOnline());

    motorOnline = false;

    EXPECT_FALSE(agitator.isOnline());
}

TEST_F(AgitatorSubsystemTest, getCurrentValue_returns_unwrapped_encoder_position)
{
    position = tap::algorithms::WrappedFloat(1.5f, -M_PI, M_PI);

    EXPECT_NEAR(1.5f, agitator.getCurrentValue(), 1E-3);
}

TEST_F(AgitatorSubsystemTest, moveToNextPosition_increments_desired_position_by_setpointIncrement)
{
    agitator.moveToNextPosition();

    // Position hasn't moved yet, so we should no longer be at the (now further away) setpoint.
    EXPECT_FALSE(agitator.atDesiredPosition());

    position = tap::algorithms::WrappedFloat(M_PI / 4, -M_PI, M_PI);

    EXPECT_TRUE(agitator.atDesiredPosition());
}

TEST_F(AgitatorSubsystemTest, atDesiredPosition_true_when_within_tolerance)
{
    // Desired position starts at 0, and the motor starts at 0, so we start at the setpoint.
    EXPECT_TRUE(agitator.atDesiredPosition());
}

TEST_F(AgitatorSubsystemTest, atDesiredPosition_false_when_outside_tolerance)
{
    position = tap::algorithms::WrappedFloat(0.5f, -M_PI, M_PI);

    EXPECT_FALSE(agitator.atDesiredPosition());
}

TEST_F(AgitatorSubsystemTest, refresh_commands_motor_output_towards_setpoint)
{
    agitator.moveToNextPosition();

    clock.time += 10;

    EXPECT_CALL(motor, setDesiredOutput(Gt(0)));

    agitator.refresh();
}

TEST_F(AgitatorSubsystemTest, refreshSafeDisconnect_zeroes_motor_output_and_resets_setpoint)
{
    agitator.moveToNextPosition();

    EXPECT_CALL(motor, setDesiredOutput(0));

    agitator.refreshSafeDisconnect();

    EXPECT_TRUE(agitator.atDesiredPosition());
}

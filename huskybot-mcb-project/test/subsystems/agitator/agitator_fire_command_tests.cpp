#include <cmath>

#include <gtest/gtest.h>

#include "tap/architecture/clock.hpp"
#include "tap/drivers.hpp"
#include "tap/mock/motor_interface_mock.hpp"

#include "algorithms/heat/heat_predictor.hpp"
#include "subsystems/agitator/agitator_fire_command.hpp"
#include "subsystems/agitator/agitator_subsystem.hpp"

using namespace testing;
using namespace huskybot::subsystems::agitator;
using namespace huskybot::algorithms::heat;

class AgitatorFireCommandTest : public Test
{
protected:
    AgitatorFireCommandTest()
        : agitator(
              &drivers,
              motor,
              {.setpointIncrement = M_PI / 4, .tolerance = 0.01f},
              {.kp = 1.0f, .maxOutput = 16000}),
          heatPredictor(drivers, HeatPredictor::HEAT_COST_17MM),
          fireCommand(agitator, heatPredictor)
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
    HeatPredictor heatPredictor;
    AgitatorFireCommand fireCommand;

    tap::algorithms::WrappedFloat position = tap::algorithms::WrappedFloat(0, -M_PI, M_PI);
    bool motorOnline = true;
};

TEST_F(AgitatorFireCommandTest, isReady_true_when_agitator_online)
{
    EXPECT_TRUE(fireCommand.isReady());
}

TEST_F(AgitatorFireCommandTest, isReady_false_when_agitator_offline)
{
    motorOnline = false;

    EXPECT_FALSE(fireCommand.isReady());
}

TEST_F(AgitatorFireCommandTest, initialize_advances_agitator_and_adds_heat)
{
    float heatBefore = heatPredictor.getCurrentHeatEstimate();

    fireCommand.initialize();

    EXPECT_FALSE(agitator.atDesiredPosition());
    EXPECT_NEAR(
        heatBefore + HeatPredictor::HEAT_COST_17MM,
        heatPredictor.getCurrentHeatEstimate(),
        1E-3);
}

TEST_F(AgitatorFireCommandTest, isFinished_true_once_agitator_reaches_new_setpoint)
{
    fireCommand.initialize();

    EXPECT_FALSE(fireCommand.isFinished());

    position = tap::algorithms::WrappedFloat(M_PI / 4, -M_PI, M_PI);

    EXPECT_TRUE(fireCommand.isFinished());
}

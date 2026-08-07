#include <gtest/gtest.h>

#include "tap/architecture/clock.hpp"
#include "tap/drivers.hpp"

#include "algorithms/heat/heat_predictor.hpp"
#include "control/governor/heat_limit_governor.hpp"

using namespace testing;
using namespace huskybot::algorithms::heat;
using namespace huskybot::control::governor;

class HeatLimitGovernorTest : public Test
{
protected:
    HeatLimitGovernorTest()
        : heatPredictor(drivers, HeatPredictor::HEAT_COST_17MM),
          governor(heatPredictor)
    {
    }

    void SetUp() override
    {
        robotData.turret.heat17 = 0;
        robotData.turret.heatLimit = 15;
        robotData.turret.coolingRate = 10;

        ON_CALL(drivers.refSerial, getRobotData).WillByDefault(ReturnRef(robotData));
    }

    tap::arch::clock::ClockStub clock;
    tap::Drivers drivers;
    HeatPredictor heatPredictor;
    HeatLimitGovernor governor;

    tap::communication::serial::RefSerialData::Rx::RobotData robotData{};
};

TEST_F(HeatLimitGovernorTest, isReady_true_when_under_heat_limit)
{
    EXPECT_TRUE(governor.isReady());
}

TEST_F(HeatLimitGovernorTest, isReady_false_once_heat_limit_would_be_exceeded)
{
    // heatLimit is 15, one 17mm shot costs 10 heat: firing once puts us at 10, and firing again
    // would put us at 20, over the limit.
    heatPredictor.fireProjectile();

    EXPECT_FALSE(governor.isReady());
}

TEST_F(HeatLimitGovernorTest, isFinished_false_when_under_heat_limit)
{
    EXPECT_FALSE(governor.isFinished());
}

TEST_F(HeatLimitGovernorTest, isFinished_true_once_heat_limit_would_be_exceeded)
{
    heatPredictor.fireProjectile();

    EXPECT_TRUE(governor.isFinished());
}

TEST_F(HeatLimitGovernorTest, isReady_true_again_after_cooling_down)
{
    heatPredictor.fireProjectile();

    EXPECT_FALSE(governor.isReady());

    clock.time += 1000;  // 1 second (ClockStub::time is in milliseconds).

    EXPECT_TRUE(governor.isReady());
}

TEST_F(HeatLimitGovernorTest, isReady_true_when_ref_data_invalid)
{
    robotData.turret.heat17 = 0xffff;
    robotData.turret.heatLimit = 0xffff;

    heatPredictor.fireProjectile();
    heatPredictor.fireProjectile();
    heatPredictor.fireProjectile();

    EXPECT_TRUE(governor.isReady());
}

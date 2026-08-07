#include <gtest/gtest.h>

#include "tap/architecture/clock.hpp"
#include "tap/drivers.hpp"

#include "algorithms/heat/heat_predictor.hpp"

using namespace testing;
using namespace huskybot::algorithms::heat;

class HeatPredictorTest : public Test
{
protected:
    HeatPredictorTest() : heatPredictor(drivers, HeatPredictor::HEAT_COST_17MM) {}

    void SetUp() override
    {
        robotData.turret.heat17 = 0;
        robotData.turret.heatLimit = 100;
        robotData.turret.coolingRate = 10;

        ON_CALL(drivers.refSerial, getRobotData).WillByDefault(ReturnRef(robotData));
    }

    tap::arch::clock::ClockStub clock;
    tap::Drivers drivers;
    HeatPredictor heatPredictor;

    tap::communication::serial::RefSerialData::Rx::RobotData robotData{};
};

TEST_F(HeatPredictorTest, fireProjectile_adds_projectile_heat_cost)
{
    heatPredictor.fireProjectile();

    EXPECT_NEAR(HeatPredictor::HEAT_COST_17MM, heatPredictor.getCurrentHeatEstimate(), 1E-3);

    heatPredictor.fireProjectile();

    EXPECT_NEAR(2 * HeatPredictor::HEAT_COST_17MM, heatPredictor.getCurrentHeatEstimate(), 1E-3);
}

TEST_F(HeatPredictorTest, updateHeatCost_cools_down_over_time)
{
    heatPredictor.fireProjectile();
    heatPredictor.updateHeatCost();

    // Half a second (ClockStub::time is in milliseconds).
    clock.time += 500;

    heatPredictor.updateHeatCost();

    // Cooled by coolingRate (units/sec) * 0.5 sec.
    EXPECT_NEAR(
        HeatPredictor::HEAT_COST_17MM - robotData.turret.coolingRate * 0.5f,
        heatPredictor.getCurrentHeatEstimate(),
        1E-1);
}

TEST_F(HeatPredictorTest, updateHeatCost_does_not_go_below_0)
{
    clock.time += 1000;

    heatPredictor.updateHeatCost();

    EXPECT_NEAR(0.0f, heatPredictor.getCurrentHeatEstimate(), 1E-3);
}

TEST_F(HeatPredictorTest, updateHeatCost_uses_ref_system_estimate_if_higher)
{
    robotData.turret.heat17 = 80;

    heatPredictor.updateHeatCost();

    EXPECT_NEAR(80.0f, heatPredictor.getCurrentHeatEstimate(), 1E-3);
}

TEST_F(HeatPredictorTest, wouldExceedHeatLimit_false_when_ref_data_invalid)
{
    robotData.turret.heat17 = 0xffff;
    robotData.turret.heatLimit = 0xffff;

    // Fire enough projectiles that we'd otherwise be over the limit.
    for (int i = 0; i < 20; i++)
    {
        heatPredictor.fireProjectile();
    }

    EXPECT_FALSE(heatPredictor.wouldExceedHeatLimit());
}

TEST_F(HeatPredictorTest, wouldExceedHeatLimit_false_when_under_limit)
{
    robotData.turret.heatLimit = 100;

    // 8 * 10 = 80 accumulated; one more shot (+10) still fits under the 100 limit.
    for (int i = 0; i < 8; i++)
    {
        heatPredictor.fireProjectile();
    }

    EXPECT_FALSE(heatPredictor.wouldExceedHeatLimit());
}

TEST_F(HeatPredictorTest, wouldExceedHeatLimit_true_when_over_limit)
{
    robotData.turret.heatLimit = 15;

    heatPredictor.fireProjectile();

    EXPECT_TRUE(heatPredictor.wouldExceedHeatLimit());
}

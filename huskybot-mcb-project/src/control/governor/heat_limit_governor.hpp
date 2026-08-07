#pragma once

#include "tap/control/governor/command_governor_interface.hpp"

#include "algorithms/heat/heat_predictor.hpp"

namespace huskybot::control::governor
{
/**
 * Governor that blocks a governed Command from running (or continuing to run) if firing another
 * projectile would put the heat estimate over the referee system's heat limit.
 */
class HeatLimitGovernor : public tap::control::governor::CommandGovernorInterface
{
public:
    explicit HeatLimitGovernor(algorithms::heat::HeatPredictor& heatPredictor);

    bool isReady() override;

    bool isFinished() override;

private:
    algorithms::heat::HeatPredictor& heatPredictor;
};
}  // namespace huskybot::control::governor

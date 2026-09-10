#pragma once

#include "tap/control/governor/command_governor_interface.hpp"

#include "algorithms/heat/heat_predictor.hpp"

namespace huskybot::control::governor
{
/**
 * Governor that prevents a command from running unless there is enough heat for another projectile.
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

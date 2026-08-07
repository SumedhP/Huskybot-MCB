#include "heat_limit_governor.hpp"

namespace huskybot::control::governor
{
HeatLimitGovernor::HeatLimitGovernor(algorithms::heat::HeatPredictor& heatPredictor)
    : heatPredictor(heatPredictor)
{
}

bool HeatLimitGovernor::isReady() { return !heatPredictor.wouldExceedHeatLimit(); }

bool HeatLimitGovernor::isFinished() { return heatPredictor.wouldExceedHeatLimit(); }
}  // namespace huskybot::control::governor

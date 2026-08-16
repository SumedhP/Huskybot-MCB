#include "heat_predictor.hpp"

#include "tap/architecture/clock.hpp"

namespace huskybot::algorithms::heat
{
HeatPredictor::HeatPredictor(tap::Drivers& drivers, float projectileHeatCost)
    : drivers(drivers),
      projectileHeatCost(projectileHeatCost),
      currentHeatEstimate(0.0f)
{
}

void HeatPredictor::updateHeatCost()
{
    uint32_t currentTime = tap::arch::clock::getTimeMicroseconds();
    uint32_t dt = (currentTime - lastUpdateTime) / 1e6;
    lastUpdateTime = currentTime;

    // Get the current turret heat from the referee data
    const auto& turretData = drivers.refSerial.getRobotData().turret;
    float coolingRate = static_cast<float>(turretData.coolingRate);
    currentHeatEstimate -= coolingRate * dt;

    // If the heat estimate of the ref system is higher than ours, we should use that instead
    float refHeatEstimate = static_cast<float>(turretData.heat17);
    if (refHeatEstimate > currentHeatEstimate)
    {
        currentHeatEstimate = refHeatEstimate;
    }

    // Clamp the heat estimate to be non-negative
    if (currentHeatEstimate < 0.0f)
    {
        currentHeatEstimate = 0.0f;
    }
}

void HeatPredictor::fireProjectile() { currentHeatEstimate += projectileHeatCost; }

float HeatPredictor::getCurrentHeatEstimate() const { return currentHeatEstimate; }

bool HeatPredictor::wouldExceedHeatLimit() const
{
    const auto& turretData = drivers.refSerial.getRobotData().turret;
    if (!tap::communication::serial::RefSerial::heatAndLimitValid(
            turretData.heat17,
            turretData.heatLimit))
    {
        return false;
    }

    return (currentHeatEstimate + projectileHeatCost) > static_cast<float>(turretData.heatLimit);
}
}  // namespace huskybot::algorithms::heat
#pragma once

#include "tap/communication/serial/ref_serial_data.hpp"
#include "tap/drivers.hpp"

namespace huskybot::algorithms::heat
{
using namespace tap::communication::serial;

class HeatPredictor
{
public:
    static constexpr float HEAT_COST_17MM = 10.0f;
    static constexpr float HEAT_COST_42MM = 100.0f;

    HeatPredictor(tap::Drivers& drivers, float projectileHeatCost);

    void updateHeatCost();

    void fireProjectile();

    float getCurrentHeatEstimate() const;

    /**
     * @return True if firing another projectile would put the heat estimate over the
     * referee system's heat limit, false if the limit is unknown (no valid ref data) or
     * firing would stay under it.
     */
    bool wouldExceedHeatLimit() const;

private:
    const tap::Drivers& drivers;
    const float projectileHeatCost;

    float currentHeatEstimate;
    uint32_t lastUpdateTime = 0;
};
}  // namespace huskybot::algorithms::heat
#pragma once

#include "tap/communication/serial/ref_serial_data.hpp"
#include "tap/drivers.hpp"

namespace algorithms::heat
{
using namespace tap::communication::serial;

class HeatPredictor
{
public:
    static constexpr float HEAT_COST_17MM = 10.0f;
    static constexpr float HEAT_COST_42MM = 100.0f;

    HeatPredictor(tap::Drivers& drivers, float projectileHeatCost);

    void updateHeatCost();

    inline void fireProjectile();

    inline float getCurrentHeatEstimate() const;

private:
    const tap::Drivers& drivers;
    const float projectileHeatCost;

    float currentHeatEstimate;
    uint32_t lastUpdateTime = 0;
};
}  // namespace algorithms::heat
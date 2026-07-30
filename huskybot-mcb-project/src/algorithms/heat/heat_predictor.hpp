#pragma once

#include "tap/architecture/clock.hpp"
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


};
}  // namespace algorithms::heat
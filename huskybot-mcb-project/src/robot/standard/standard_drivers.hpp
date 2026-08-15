#pragma once

#include "tap/drivers.hpp"

#include "algorithms/heat/heat_predictor.hpp"
#include "control/control_operator_interface.hpp"

#include "standard_constants.hpp"

namespace huskybot::standard
{
/**
 * The standard's drivers: taproot's hardware drivers plus the robot-wide state that isn't tied to
 * a single subsystem. Anything a subsystem owns belongs in that subsystem, not here.
 */
class Drivers : public tap::Drivers
{
    friend class DriversSingleton;

#ifdef ENV_UNIT_TESTS
public:
#endif
    Drivers() : tap::Drivers(), heatPredictor(*this, constants::PROJECTILE_HEAT_COST) {}

public:
    control::ControlOperatorInterface controlOperatorInterface;
    algorithms::heat::HeatPredictor heatPredictor;
};  // class Drivers

}  // namespace huskybot::standard

#ifndef ENV_UNIT_TESTS

#include "drivers_singleton.hpp"

namespace tap
{
/**
 * `tap::Drivers`' constructor is protected and befriends `tap::DriversSingleton`, so the one
 * static instance has to be declared here, in taproot's namespace.
 */
class DriversSingleton
{
public:
    static Drivers drivers;
};  // class DriversSingleton

Drivers DriversSingleton::drivers;
}  // namespace tap

namespace huskybot
{
tap::Drivers *DoNotUse_getDrivers() { return &tap::DriversSingleton::drivers; }
}  // namespace huskybot

#endif

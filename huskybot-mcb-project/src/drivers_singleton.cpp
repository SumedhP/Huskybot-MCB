#ifndef ENV_UNIT_TESTS

#include "drivers_singleton.hpp"

#if defined(TARGET_STANDARD)
namespace huskybot::standard
#endif
{
/**
 * Class that allows one to construct a Drivers instance because of friendship
 * with the Drivers class.
 */
class DriversSingleton
{
public:
    static Drivers drivers;
};  // class DriversSingleton

Drivers DriversSingleton::drivers;

Drivers *DoNotUse_getDrivers() { return &DriversSingleton::drivers; }
}  // namespace huskybot::<robot>

#endif

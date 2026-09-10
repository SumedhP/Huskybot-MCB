#pragma once

#ifndef ENV_UNIT_TESTS

#include "tap/drivers.hpp"

namespace huskybot
{
/**
 * @return The singleton instance of the Drivers class. This is the only instance of the
 *      Drivers class that should be created anywhere in the non-unit test framework.
 * @note It is likely that you will never have to use this. There are only two files you
 *      should be calling this function from -- `main.cpp` and `*_control.cpp`, either to
 *      run I/O stuff and to add a Drivers pointer to an instance of a Subsystem or Command.
 */
tap::Drivers *DoNotUse_getDrivers();
using driversFunc = tap::Drivers *(*)();
}  // namespace huskybot

#endif  // ENV_UNIT_TESTS

#pragma once

#include "tap/drivers.hpp"

namespace huskybot::util
{
using tap::control::SafeDisconnectFunction;

class RemoteSafeDisconnectFunction : public SafeDisconnectFunction
{
public:
    RemoteSafeDisconnectFunction(tap::Drivers *drivers) : drivers(drivers) {}

    bool operator()() override { return !drivers->remote.isConnected(); }

private:
    tap::Drivers *drivers;
};
}  // namespace huskybot::util

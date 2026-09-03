#pragma once

#include <cstdint>

#include "tap/architecture/clock.hpp"

namespace huskybot::util
{
/**
 * Measures the time elapsed between successive calls.
 *
 * Intended for control loops that require precise timing between loops.
 */
class DeltaTime
{
public:
    static constexpr float INITIAL_DT = 0.0f;

    /**
     * @return Seconds since the previous call. Returns `INITIAL_DT` on the first call or following `restart()`.
     */
    float getTimeElapsed()
    {
        uint32_t currentTime = tap::arch::clock::getTimeMicroseconds();
        float dt = running ? (currentTime - lastTime) / 1e6f : INITIAL_DT;
        lastTime = currentTime;
        running = true;
        return dt;
    }

    void restart() {
        running = false;
    }

private:
    uint32_t lastTime = 0;
    bool running = false;
};

}  // namespace huskybot::util

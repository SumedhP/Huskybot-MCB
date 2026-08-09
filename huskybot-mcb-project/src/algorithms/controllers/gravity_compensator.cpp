#include "gravity_compensator.hpp"

#include <cmath>

#include "tap/algorithms/math_user_utils.hpp"

namespace huskybot::algorithms::controllers
{
GravityCompensator::GravityCompensator(const GravityCompensatorConfig& config) : config(config) {}

float GravityCompensator::calculateEffort(float pitchWorldFrame) const
{
    bool cgXZero = tap::algorithms::compareFloatClose(config.cgX, 0.0f, 1E-5f);
    bool cgZZero = tap::algorithms::compareFloatClose(config.cgZ, 0.0f, 1E-5f);

    // CG is at the pivot, so there's no lever arm for gravity to act on.
    if (cgXZero && cgZZero)
    {
        return 0.0f;
    }

    float cgAngle = atan2f(config.cgZ, config.cgX);
    return config.maxCompensationOutput * cosf(cgAngle - pitchWorldFrame);
}
}  // namespace huskybot::algorithms::controllers

#pragma once

namespace huskybot::algorithms::controllers
{
struct GravityCompensatorConfig
{
    /// Forward offset of the pitch assembly's center of gravity from the pitch pivot, in mm.
    /// Positive is forward.
    float cgX = 0.0f;
    /// Vertical offset of the pitch assembly's center of gravity from the pitch pivot, in mm.
    /// Positive is up.
    float cgZ = 0.0f;
    /// The motor output needed to fully cancel gravity when the CG is level with the pivot
    /// (where gravitational torque is maximal).
    float maxCompensationOutput = 0.0f;
};

/**
 * Computes the open-loop motor output needed to counteract gravity's torque on the pitch axis,
 * as a function of world-frame pitch angle and the pitch assembly's center of gravity.
 */
class GravityCompensator
{
public:
    explicit GravityCompensator(const GravityCompensatorConfig& config);

    /**
     * @param pitchWorldFrame The world-frame pitch angle, in radians.
     * @return The motor output needed to counteract gravity at this pitch angle.
     */
    float calculateEffort(float pitchWorldFrame) const;

private:
    GravityCompensatorConfig config;
};

}  // namespace huskybot::algorithms::controllers

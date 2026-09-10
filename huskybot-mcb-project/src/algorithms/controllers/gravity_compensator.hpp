#pragma once

namespace huskybot::algorithms::controllers
{
/**
 * Configuration for gravity compensation, specifying the position of the center of gravity
 * relative to the turret's pitch axis. gravityCompensationScalar can be considered feedforward
 * gain based on the turret angle.
 */
struct GravityCompensatorConfig
{
    float cgX = 0.0f;
    float cgZ = 0.0f;
    float gravityCompensationScalar = 0.0f;
};

/**
 * Computes the motor output needed to counteract gravity's torque on the pitch axis.
 */
class GravityCompensator
{
public:
    explicit GravityCompensator(const GravityCompensatorConfig& config);

    /**
     * @param pitch The world-frame pitch angle, in radians.
     * @return The motor output needed to counteract gravity at this pitch angle.
     */
    float calculateEffort(float pitch) const;

private:
    GravityCompensatorConfig config;
};

}  // namespace huskybot::algorithms::controllers

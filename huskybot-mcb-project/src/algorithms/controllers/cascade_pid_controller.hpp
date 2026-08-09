#pragma once

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/algorithms/wrapped_float.hpp"

namespace huskybot::algorithms::controllers
{
struct CascadePidControllerConfig
{
    /// Outer loop: converts angle error into a desired velocity.
    tap::algorithms::SmoothPidConfig positionPidConfig;
    /// Inner loop: converts velocity error into motor output.
    tap::algorithms::SmoothPidConfig velocityPidConfig;
    /// Gain applied to the feedforward velocity, added directly to the output.
    float feedforwardGain = 0.0f;
};

/**
 * A generic position -> velocity cascade PID controller for a single wrapped-angle axis, with an
 * open-loop velocity feedforward term added directly to the output.
 */
class CascadePidController
{
public:
    explicit CascadePidController(const CascadePidControllerConfig& config);

    /**
     * Runs one iteration of the cascade controller.
     *
     * @param setpointAngle The desired angle, in radians.
     * @param measuredAngle The measured angle, in radians.
     * @param measuredVelocity The measured velocity, in radians/second.
     * @param feedforwardVelocity The open-loop feedforward velocity, in radians/second.
     * @param dt The time since this function was last called, in seconds.
     * @return The motor output.
     */
    float runController(
        const tap::algorithms::WrappedFloat& setpointAngle,
        const tap::algorithms::WrappedFloat& measuredAngle,
        float measuredVelocity,
        float feedforwardVelocity,
        float dt);

private:
    float feedforwardGain;
    tap::algorithms::SmoothPid positionPid;
    tap::algorithms::SmoothPid velocityPid;
};

}  // namespace huskybot::algorithms::controllers

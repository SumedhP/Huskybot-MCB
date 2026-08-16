#include "cascade_pid_controller.hpp"

namespace huskybot::algorithms::controllers
{
CascadePidController::CascadePidController(const CascadePidControllerConfig& config)
    : feedforwardGain(config.feedforwardGain),
      positionPid(config.positionPidConfig),
      velocityPid(config.velocityPidConfig)
{
}

float CascadePidController::runController(
    const tap::algorithms::WrappedFloat& setpointAngle,
    const tap::algorithms::WrappedFloat& measuredAngle,
    float measuredVelocity,
    float inputVelocity,
    float dt)
{
    float positionError = measuredAngle.minDifference(setpointAngle);
    float desiredVelocity = positionPid.runController(positionError, measuredVelocity, dt);

    float velocityError = desiredVelocity - measuredVelocity;
    float feedback = velocityPid.runControllerDerivateError(velocityError, dt);

    return feedback + inputVelocity * feedforwardGain;
}
}  // namespace huskybot::algorithms::controllers

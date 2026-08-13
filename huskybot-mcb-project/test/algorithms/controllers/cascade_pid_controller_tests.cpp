#include <cmath>

#include <gtest/gtest.h>

#include "algorithms/controllers/cascade_pid_controller.hpp"

using namespace huskybot::algorithms::controllers;

static tap::algorithms::WrappedFloat angle(float value)
{
    return tap::algorithms::WrappedFloat(value, -M_PI, M_PI);
}

TEST(CascadePidControllerTest, feedforward_passes_through_when_pid_gains_are_zero)
{
    CascadePidController controller(
        {.positionPidConfig = {}, .velocityPidConfig = {}, .feedforwardGain = 2.0f});

    float output = controller.runController(angle(0.0f), angle(0.0f), 0.0f, 3.0f, 0.01f);

    EXPECT_FLOAT_EQ(6.0f, output);
}

TEST(CascadePidControllerTest, positive_position_error_drives_positive_output)
{
    CascadePidController controller(
        {.positionPidConfig = {.kp = 1.0f, .maxOutput = 100.0f},
         .velocityPidConfig = {.kp = 1.0f, .maxOutput = 100.0f},
         .feedforwardGain = 0.0f});

    // Setpoint is ahead of the measured angle, and we're not yet moving towards it.
    float output = controller.runController(angle(1.0f), angle(0.0f), 0.0f, 0.0f, 0.01f);

    EXPECT_GT(output, 0.0f);
}

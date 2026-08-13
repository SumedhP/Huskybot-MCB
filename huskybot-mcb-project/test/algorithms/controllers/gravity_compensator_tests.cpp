#define _USE_MATH_DEFINES
#include <cmath>

#include <gtest/gtest.h>

#include "algorithms/controllers/gravity_compensator.hpp"

using namespace huskybot::algorithms::controllers;

TEST(GravityCompensatorTest, zero_cg_returns_zero_effort_at_any_pitch)
{
    GravityCompensator compensator({.cgX = 0.0f, .cgZ = 0.0f, .maxCompensationOutput = 5000.0f});

    EXPECT_FLOAT_EQ(0.0f, compensator.calculateEffort(0.0f));
    EXPECT_FLOAT_EQ(0.0f, compensator.calculateEffort(1.0f));
}

TEST(GravityCompensatorTest, cg_directly_forward_peaks_at_level_pitch)
{
    // CG straight out in front of the pivot (cgZ = 0): max effort when level (pitch = 0).
    GravityCompensator compensator({.cgX = 30.0f, .cgZ = 0.0f, .maxCompensationOutput = 5000.0f});

    EXPECT_NEAR(5000.0f, compensator.calculateEffort(0.0f), 1E-2);
    EXPECT_NEAR(0.0f, compensator.calculateEffort(M_PI / 2), 1E-2);
}

TEST(GravityCompensatorTest, cg_offset_angle_shifts_where_effort_peaks)
{
    // CG at 45 degrees above forward: effort peaks when pitched down to align the CG horizontally.
    GravityCompensator compensator({.cgX = 1.0f, .cgZ = 1.0f, .maxCompensationOutput = 5000.0f});

    EXPECT_NEAR(5000.0f, compensator.calculateEffort(M_PI / 4), 1E-2);
}

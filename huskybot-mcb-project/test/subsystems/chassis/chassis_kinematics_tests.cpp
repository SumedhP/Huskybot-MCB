#include <gtest/gtest.h>

#include "subsystems/chassis/chassis_kinematics.hpp"

using namespace huskybot::subsystems::chassis;

static constexpr ChassisGeometry GEOMETRY{.wheelRadius = 0.076f, .lengthX = 0.2f, .lengthY = 0.15f};

/// Runs the inverse kinematics, then the least-squares forward kinematics, and checks the chassis
/// velocity survives the round trip.
static void expectRoundTrip(const WheelMatrix& wheelMatrix, const ChassisVelocity& velocity)
{
    tap::algorithms::CMSISMat<3, NUM_WHEELS> chassisMatrix =
        (wheelMatrix.transpose() * wheelMatrix).inverse() * wheelMatrix.transpose();

    tap::algorithms::CMSISMat<3, 1> result =
        chassisMatrix *
        (wheelMatrix * tap::algorithms::CMSISMat<3, 1>({velocity.x, velocity.y, velocity.r}));

    EXPECT_NEAR(velocity.x, result[0], 1e-4);
    EXPECT_NEAR(velocity.y, result[1], 1e-4);
    EXPECT_NEAR(velocity.r, result[2], 1e-4);
}

TEST(ChassisKinematicsTest, mecanum_drives_all_wheels_forward_together)
{
    WheelMatrix wheels = mecanumWheelMatrix(GEOMETRY);
    tap::algorithms::CMSISMat<NUM_WHEELS, 1> speeds =
        wheels * tap::algorithms::CMSISMat<3, 1>({1.0f, 0.0f, 0.0f});

    const float expected = 1.0f / GEOMETRY.wheelRadius;
    EXPECT_NEAR(expected, speeds[LEFT_FRONT], 1e-4);
    EXPECT_NEAR(expected, speeds[RIGHT_FRONT], 1e-4);
    EXPECT_NEAR(expected, speeds[LEFT_BACK], 1e-4);
    EXPECT_NEAR(expected, speeds[RIGHT_BACK], 1e-4);
}

TEST(ChassisKinematicsTest, mecanum_rotation_runs_the_left_side_against_the_right)
{
    WheelMatrix wheels = mecanumWheelMatrix(GEOMETRY);
    tap::algorithms::CMSISMat<NUM_WHEELS, 1> speeds =
        wheels * tap::algorithms::CMSISMat<3, 1>({0.0f, 0.0f, 1.0f});

    // Counter-clockwise means the left wheels run backwards and the right wheels run forwards.
    EXPECT_LT(speeds[LEFT_FRONT], 0.0f);
    EXPECT_LT(speeds[LEFT_BACK], 0.0f);
    EXPECT_GT(speeds[RIGHT_FRONT], 0.0f);
    EXPECT_GT(speeds[RIGHT_BACK], 0.0f);
}

TEST(ChassisKinematicsTest, omni_spins_all_wheels_the_same_way_to_rotate)
{
    WheelMatrix wheels = omniWheelMatrix(GEOMETRY);
    tap::algorithms::CMSISMat<NUM_WHEELS, 1> speeds =
        wheels * tap::algorithms::CMSISMat<3, 1>({0.0f, 0.0f, 1.0f});

    const float armLength =
        sqrtf(GEOMETRY.lengthX * GEOMETRY.lengthX + GEOMETRY.lengthY * GEOMETRY.lengthY);
    const float expected = armLength / GEOMETRY.wheelRadius;

    for (int wheel = 0; wheel < NUM_WHEELS; wheel++)
    {
        EXPECT_NEAR(expected, speeds[wheel], 1e-4);
    }
}

TEST(ChassisKinematicsTest, omni_forward_thrust_cancels_sideways_and_rotation)
{
    WheelMatrix wheels = omniWheelMatrix(GEOMETRY);
    tap::algorithms::CMSISMat<NUM_WHEELS, 1> speeds =
        wheels * tap::algorithms::CMSISMat<3, 1>({1.0f, 0.0f, 0.0f});

    // Pure forward motion has to leave the wheels balanced, or the chassis would spin.
    EXPECT_NEAR(
        0.0f,
        speeds[LEFT_FRONT] + speeds[RIGHT_FRONT] + speeds[LEFT_BACK] + speeds[RIGHT_BACK],
        1e-4);
    EXPECT_NEAR(-speeds[LEFT_FRONT], speeds[RIGHT_FRONT], 1e-4);
}

TEST(ChassisKinematicsTest, mecanum_survives_a_round_trip_through_both_directions)
{
    expectRoundTrip(mecanumWheelMatrix(GEOMETRY), {.x = 1.5f, .y = -0.75f, .r = 2.0f});
}

TEST(ChassisKinematicsTest, omni_survives_a_round_trip_through_both_directions)
{
    expectRoundTrip(omniWheelMatrix(GEOMETRY), {.x = 1.5f, .y = -0.75f, .r = 2.0f});
}

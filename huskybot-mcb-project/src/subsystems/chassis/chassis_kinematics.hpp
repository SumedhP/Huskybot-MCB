#pragma once

#include "tap/algorithms/cmsis_mat.hpp"

namespace huskybot::subsystems::chassis
{
/// The wheels of a four-wheel holonomic chassis, in the order used by every wheel matrix.
enum WheelIndex
{
    LEFT_FRONT = 0,
    RIGHT_FRONT,
    LEFT_BACK,
    RIGHT_BACK,
    NUM_WHEELS,
};

/**
 * Inverse kinematics for a four-wheel holonomic chassis: maps a chassis-frame velocity to the
 * four wheel angular velocities that produce it, as `wheelSpeeds = matrix * {x, y, r}`.
 *
 * The forward direction (wheel speeds back to a chassis velocity) is the pseudoinverse of this,
 * which `ChassisSubsystem` computes for you, so a drive type is fully described by its matrix.
 */
using WheelMatrix = tap::algorithms::CMSISMat<NUM_WHEELS, 3>;

/**
 * A holonomic chassis velocity, in the chassis frame (x forward, y left, r counter-clockwise).
 */
struct ChassisVelocity
{
    float x = 0.0f;  /// Forward velocity, in meters/second.
    float y = 0.0f;  /// Leftward velocity, in meters/second.
    float r = 0.0f;  /// Counter-clockwise rotational velocity, in radians/second.
};

/**
 * The physical dimensions a wheel matrix is built from.
 */
struct ChassisGeometry
{
    float wheelRadius = 0.0f;  /// Radius of a wheel, in meters.
    float lengthX = 0.0f;      /// Distance from the chassis center to a wheel's axle, forward, in
                               /// meters (half the wheelbase).
    float lengthY = 0.0f;      /// Distance from the chassis center to a wheel's axle, sideways, in
                               /// meters (half the track width).
};

/**
 * Builds the wheel matrix for four mecanum wheels, each with its rollers at 45 degrees and mounted
 * mirrored front-to-back and left-to-right (the standard layout).
 *
 * Positive wheel velocity drives the chassis forward.
 */
WheelMatrix mecanumWheelMatrix(const ChassisGeometry& geometry);

/**
 * Builds the wheel matrix for four omni wheels in an X configuration: each wheel sits 45 degrees
 * off the chassis axes and rolls tangent to the circle around the chassis center.
 *
 * Positive wheel velocity drives the chassis counter-clockwise. `lengthX` and `lengthY` give the
 * wheel's position, so the wheels do not have to sit on a perfect 45 degree diagonal.
 */
WheelMatrix omniWheelMatrix(const ChassisGeometry& geometry);

}  // namespace huskybot::subsystems::chassis

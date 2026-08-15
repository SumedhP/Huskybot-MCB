#include "chassis_kinematics.hpp"

#include <cmath>

namespace huskybot::subsystems::chassis
{
WheelMatrix mecanumWheelMatrix(const ChassisGeometry& geometry)
{
    // A mecanum wheel can only push along its rollers, so a wheel's contribution to sideways
    // motion and to rotation both flip sign with the roller's 45 degree mounting direction.
    const float inverseRadius = 1.0f / geometry.wheelRadius;
    const float rotationArm = geometry.lengthX + geometry.lengthY;

    return WheelMatrix({
        inverseRadius,
        -inverseRadius,
        -inverseRadius * rotationArm,  // left front
        inverseRadius,
        inverseRadius,
        inverseRadius * rotationArm,  // right front
        inverseRadius,
        inverseRadius,
        -inverseRadius * rotationArm,  // left back
        inverseRadius,
        -inverseRadius,
        inverseRadius * rotationArm,  // right back
    });
}

WheelMatrix omniWheelMatrix(const ChassisGeometry& geometry)
{
    // Each wheel rolls tangent to the circle through all four wheels, so it sees the component of
    // the chassis velocity along that tangent, plus the whole rotation scaled by its lever arm.
    const float inverseRadius = 1.0f / geometry.wheelRadius;
    const float rotationArm =
        sqrtf(geometry.lengthX * geometry.lengthX + geometry.lengthY * geometry.lengthY);

    const float tangentX = inverseRadius * geometry.lengthY / rotationArm;
    const float tangentY = inverseRadius * geometry.lengthX / rotationArm;
    const float rotation = inverseRadius * rotationArm;

    return WheelMatrix({
        -tangentX,
        tangentY,
        rotation,  // left front
        tangentX,
        tangentY,
        rotation,  // right front
        -tangentX,
        -tangentY,
        rotation,  // left back
        tangentX,
        -tangentY,
        rotation,  // right back
    });
}
}  // namespace huskybot::subsystems::chassis

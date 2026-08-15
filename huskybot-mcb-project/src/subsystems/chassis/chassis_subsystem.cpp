#include "chassis_subsystem.hpp"

#include "tap/architecture/clock.hpp"

namespace huskybot::subsystems::chassis
{
ChassisSubsystem::ChassisSubsystem(
    tap::Drivers* drivers,
    tap::motor::MotorInterface& leftFrontMotor,
    tap::motor::MotorInterface& rightFrontMotor,
    tap::motor::MotorInterface& leftBackMotor,
    tap::motor::MotorInterface& rightBackMotor,
    const WheelMatrix& wheelMatrix,
    const tap::algorithms::SmoothPidConfig& pidConfig)
    : Subsystem(drivers),
      motors{&leftFrontMotor, &rightFrontMotor, &leftBackMotor, &rightBackMotor},
      wheelPids{
          tap::algorithms::SmoothPid(pidConfig),
          tap::algorithms::SmoothPid(pidConfig),
          tap::algorithms::SmoothPid(pidConfig),
          tap::algorithms::SmoothPid(pidConfig)},
      wheelMatrix(wheelMatrix),
      // Four wheels give more measurements than the three degrees of freedom they describe, so the
      // wheel matrix has no true inverse. Its left pseudoinverse is the least-squares fit, and the
      // geometry never changes, so it is worth paying for once here.
      chassisMatrix((wheelMatrix.transpose() * wheelMatrix).inverse() * wheelMatrix.transpose())
{
}

void ChassisSubsystem::initialize()
{
    for (auto motor : motors)
    {
        motor->initialize();
    }
}

void ChassisSubsystem::refresh()
{
    uint32_t currentTime = tap::arch::clock::getTimeMicroseconds();
    float dt = (currentTime - lastRefreshTime) / 1e6f;
    lastRefreshTime = currentTime;

    tap::algorithms::CMSISMat<NUM_WHEELS, 1> desiredWheelSpeeds =
        wheelMatrix *
        tap::algorithms::CMSISMat<3, 1>({desiredVelocity.x, desiredVelocity.y, desiredVelocity.r});

    for (int wheel = 0; wheel < NUM_WHEELS; wheel++)
    {
        float error = desiredWheelSpeeds[wheel] - motors[wheel]->getEncoder()->getVelocity();
        float output = wheelPids[wheel].runControllerDerivateError(error, dt);
        motors[wheel]->setDesiredOutput(static_cast<int32_t>(output));
    }
}

void ChassisSubsystem::refreshSafeDisconnect()
{
    desiredVelocity = ChassisVelocity();

    for (auto motor : motors)
    {
        motor->setDesiredOutput(0);
    }

    lastRefreshTime = tap::arch::clock::getTimeMicroseconds();
}

void ChassisSubsystem::setDesiredVelocity(const ChassisVelocity& velocity)
{
    desiredVelocity = velocity;
}

ChassisVelocity ChassisSubsystem::getDesiredVelocity() const { return desiredVelocity; }

ChassisVelocity ChassisSubsystem::getCurrentVelocity() const
{
    tap::algorithms::CMSISMat<NUM_WHEELS, 1> wheelSpeeds;

    for (int wheel = 0; wheel < NUM_WHEELS; wheel++)
    {
        wheelSpeeds[wheel] = motors[wheel]->getEncoder()->getVelocity();
    }

    tap::algorithms::CMSISMat<3, 1> velocity = chassisMatrix * wheelSpeeds;
    return ChassisVelocity{velocity[0], velocity[1], velocity[2]};
}

bool ChassisSubsystem::isOnline() const
{
    for (auto motor : motors)
    {
        if (!motor->isMotorOnline())
        {
            return false;
        }
    }

    return true;
}
}  // namespace huskybot::subsystems::chassis

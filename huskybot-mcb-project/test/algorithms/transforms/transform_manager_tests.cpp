#include <cmath>

#include <gtest/gtest.h>

#include "tap/drivers.hpp"
#include "tap/mock/abstract_imu_mock.hpp"
#include "tap/mock/motor_interface_mock.hpp"

#include "algorithms/transforms/transform_manager.hpp"

using namespace testing;
using namespace huskybot::algorithms::transforms;
using namespace huskybot::subsystems::turret;

class TransformManagerTest : public Test
{
protected:
    TransformManagerTest()
        : turret(
              &drivers,
              yawMotor,
              pitchMotor,
              imu,
              {.chassisFrameZeroOffset = 0.0f},
              {.chassisFrameZeroOffset = 0.0f}),
          transforms(turret)
    {
    }

    void SetUp() override
    {
        ON_CALL(*yawMotor.getEncoder(), getPosition).WillByDefault(ReturnPointee(&yawPosition));
        ON_CALL(*yawMotor.getEncoder(), getVelocity).WillByDefault(ReturnPointee(&yawVelocity));
        ON_CALL(*pitchMotor.getEncoder(), getPosition).WillByDefault(ReturnPointee(&pitchPosition));
        ON_CALL(*pitchMotor.getEncoder(), getVelocity).WillByDefault(ReturnPointee(&pitchVelocity));

        ON_CALL(imu, getYaw).WillByDefault(ReturnPointee(&imuYaw));
        ON_CALL(imu, getPitch).WillByDefault(ReturnPointee(&imuPitch));
        ON_CALL(imu, getGz).WillByDefault(ReturnPointee(&imuYawVelocity));
        ON_CALL(imu, getGy).WillByDefault(ReturnPointee(&imuPitchVelocity));
    }

    tap::Drivers drivers;
    NiceMock<tap::mock::MotorInterfaceMock> yawMotor;
    NiceMock<tap::mock::MotorInterfaceMock> pitchMotor;
    NiceMock<tap::mock::AbstractImuMock> imu;
    TurretSubsystem turret;
    TransformManager transforms;

    tap::algorithms::WrappedFloat yawPosition = tap::algorithms::Angle(0);
    tap::algorithms::WrappedFloat pitchPosition = tap::algorithms::Angle(0);
    float yawVelocity = 0.0f;
    float pitchVelocity = 0.0f;

    float imuYaw = 0.0f;
    float imuPitch = 0.0f;
    float imuYawVelocity = 0.0f;
    float imuPitchVelocity = 0.0f;
};

TEST_F(TransformManagerTest, chassis_heading_is_world_turret_heading_minus_turret_encoder)
{
    imuYaw = M_PI / 2;
    yawPosition = tap::algorithms::Angle(M_PI / 6);

    transforms.update();

    EXPECT_NEAR(M_PI / 3, transforms.getWorldToChassis().getYaw(), 1e-5);
    EXPECT_NEAR(M_PI / 6, transforms.getChassisToTurret().getYaw(), 1e-5);
    EXPECT_NEAR(M_PI / 2, transforms.getWorldToTurret().getYaw(), 1e-5);
}

TEST_F(TransformManagerTest, chassis_yaw_velocity_is_imu_velocity_minus_encoder_velocity)
{
    imuYawVelocity = 3.0f;
    yawVelocity = 1.0f;

    transforms.update();

    EXPECT_NEAR(2.0f, transforms.getWorldToChassis().getYawVelocity(), 1e-5);
    EXPECT_NEAR(1.0f, transforms.getChassisToTurret().getYawVelocity(), 1e-5);
}

TEST_F(TransformManagerTest, turret_pitch_comes_from_the_pitch_encoder)
{
    pitchPosition = tap::algorithms::Angle(M_PI / 8);

    transforms.update();

    EXPECT_NEAR(M_PI / 8, transforms.getChassisToTurret().getPitch(), 1e-5);
    EXPECT_NEAR(M_PI / 8, transforms.getWorldToTurret().getPitch(), 1e-5);
}

TEST_F(TransformManagerTest, every_frame_sits_at_the_world_origin)
{
    transforms.update();

    EXPECT_NEAR(0.0f, transforms.getWorldToChassis().getX(), 1e-5);
    EXPECT_NEAR(0.0f, transforms.getWorldToChassis().getY(), 1e-5);
    EXPECT_NEAR(0.0f, transforms.getWorldToChassis().getZ(), 1e-5);
    EXPECT_NEAR(0.0f, transforms.getWorldToTurret().getX(), 1e-5);
    EXPECT_NEAR(0.0f, transforms.getWorldToTurret().getY(), 1e-5);
    EXPECT_NEAR(0.0f, transforms.getWorldToTurret().getZ(), 1e-5);
}

#include <cmath>

#include <gtest/gtest.h>

#include "tap/drivers.hpp"
#include "tap/mock/abstract_imu_mock.hpp"
#include "tap/mock/motor_interface_mock.hpp"

#include "subsystems/turret/turret_subsystem.hpp"

using namespace testing;
using namespace huskybot::subsystems::turret;

class TurretSubsystemTest : public Test
{
protected:
    TurretSubsystemTest()
        : turret(
              &drivers,
              yawMotor,
              pitchMotor,
              imu,
              {.chassisFrameZeroOffset = M_PI / 2},
              {.chassisFrameZeroOffset = 0.0f})
    {
    }

    void SetUp() override
    {
        ON_CALL(yawMotor, isMotorOnline).WillByDefault(ReturnPointee(&yawMotorOnline));
        ON_CALL(*yawMotor.getEncoder(), getPosition).WillByDefault(ReturnPointee(&yawPosition));
        ON_CALL(*yawMotor.getEncoder(), getVelocity).WillByDefault(ReturnPointee(&yawVelocity));

        ON_CALL(pitchMotor, isMotorOnline).WillByDefault(ReturnPointee(&pitchMotorOnline));
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

    tap::algorithms::WrappedFloat yawPosition = tap::algorithms::WrappedFloat(0, -M_PI, M_PI);
    tap::algorithms::WrappedFloat pitchPosition = tap::algorithms::WrappedFloat(0, -M_PI, M_PI);
    float yawVelocity = 0.0f;
    float pitchVelocity = 0.0f;
    bool yawMotorOnline = true;
    bool pitchMotorOnline = true;

    float imuYaw = 0.0f;
    float imuPitch = 0.0f;
    float imuYawVelocity = 0.0f;
    float imuPitchVelocity = 0.0f;
};

TEST_F(TurretSubsystemTest, initialize_initializes_both_motors)
{
    EXPECT_CALL(yawMotor, initialize);
    EXPECT_CALL(pitchMotor, initialize);

    turret.initialize();
}

TEST_F(TurretSubsystemTest, isOnline_false_when_either_motor_offline)
{
    EXPECT_TRUE(turret.isOnline());

    yawMotorOnline = false;
    EXPECT_FALSE(turret.isOnline());

    yawMotorOnline = true;
    pitchMotorOnline = false;
    EXPECT_FALSE(turret.isOnline());
}

TEST_F(TurretSubsystemTest, getChassisFrameYaw_subtracts_configured_zero_offset)
{
    yawPosition = tap::algorithms::WrappedFloat(M_PI / 2, -M_PI, M_PI);

    EXPECT_NEAR(0.0f, turret.getChassisFrameYaw().getWrappedValue(), 1E-3);
}

TEST_F(TurretSubsystemTest, getWorldFrameYaw_passes_through_imu_reading)
{
    imuYaw = 1.2f;

    EXPECT_NEAR(1.2f, turret.getWorldFrameYaw().getWrappedValue(), 1E-3);
    EXPECT_FLOAT_EQ(0.0f, turret.getWorldFrameYawVelocity());

    imuYawVelocity = 0.5f;
    EXPECT_FLOAT_EQ(0.5f, turret.getWorldFrameYawVelocity());
}

TEST_F(TurretSubsystemTest, setYawMotorOutput_passes_output_through_to_motor)
{
    EXPECT_CALL(yawMotor, setDesiredOutput(1234));

    turret.setYawMotorOutput(1234);
}

TEST_F(TurretSubsystemTest, refreshSafeDisconnect_zeroes_both_motor_outputs)
{
    EXPECT_CALL(yawMotor, setDesiredOutput(0));
    EXPECT_CALL(pitchMotor, setDesiredOutput(0));

    turret.refreshSafeDisconnect();
}

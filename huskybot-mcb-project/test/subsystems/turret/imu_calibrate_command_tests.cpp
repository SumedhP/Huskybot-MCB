#include <gtest/gtest.h>

#include "tap/architecture/clock.hpp"
#include "tap/drivers.hpp"
#include "tap/mock/abstract_imu_mock.hpp"
#include "tap/mock/motor_interface_mock.hpp"

#include "subsystems/turret/imu_calibrate_command.hpp"

using namespace testing;
using namespace huskybot::subsystems::turret;
using huskybot::algorithms::controllers::CascadePidController;
using tap::communication::sensors::imu::ImuInterface;

namespace
{
/// The taproot mock doesn't cover the calibration half of the IMU interface, so track it here.
class CalibratableImuMock : public NiceMock<tap::mock::AbstractImuMock>
{
public:
    void requestCalibration() override { calibrationRequests++; }
    ImuState getImuState() const override { return state; }

    int calibrationRequests = 0;
    ImuState state = ImuState::IMU_NOT_CALIBRATED;
};
}  // namespace

class ImuCalibrateCommandTest : public Test
{
protected:
    ImuCalibrateCommandTest()
        : turret(
              &drivers,
              yawMotor,
              pitchMotor,
              imu,
              {.chassisFrameZeroOffset = 0.0f},
              {.chassisFrameZeroOffset = 0.0f}),
          yawController(
              {.positionPidConfig = {.kp = 1000.0f, .maxOutput = 20000.0f},
               .velocityPidConfig = {.kp = 1000.0f, .maxOutput = 20000.0f}}),
          pitchController(
              {.positionPidConfig = {.kp = 1000.0f, .maxOutput = 20000.0f},
               .velocityPidConfig = {.kp = 1000.0f, .maxOutput = 20000.0f}}),
          command(
              turret,
              imu,
              yawController,
              pitchController,
              {.levelPitch = 0.0f,
               .positionTolerance = 0.02f,
               .velocityTolerance = 0.05f,
               .settleTimeout = 5000,
               .calibrationTimeout = 6000})
    {
    }

    void SetUp() override
    {
        ON_CALL(*yawMotor.getEncoder(), getPosition).WillByDefault(ReturnPointee(&yawPosition));
        ON_CALL(*yawMotor.getEncoder(), getVelocity).WillByDefault(ReturnPointee(&yawVelocity));
        ON_CALL(*pitchMotor.getEncoder(), getPosition).WillByDefault(ReturnPointee(&pitchPosition));
        ON_CALL(*pitchMotor.getEncoder(), getVelocity).WillByDefault(ReturnPointee(&pitchVelocity));
        ON_CALL(yawMotor, isMotorOnline).WillByDefault(Return(true));
        ON_CALL(pitchMotor, isMotorOnline).WillByDefault(Return(true));

        clock.time = 0;
    }

    tap::arch::clock::ClockStub clock;
    tap::Drivers drivers;
    NiceMock<tap::mock::MotorInterfaceMock> yawMotor;
    NiceMock<tap::mock::MotorInterfaceMock> pitchMotor;
    CalibratableImuMock imu;
    TurretSubsystem turret;
    CascadePidController yawController;
    CascadePidController pitchController;
    ImuCalibrateCommand command;

    tap::algorithms::WrappedFloat yawPosition = tap::algorithms::Angle(0);
    tap::algorithms::WrappedFloat pitchPosition = tap::algorithms::Angle(0);
    float yawVelocity = 0.0f;
    float pitchVelocity = 0.0f;
};

TEST_F(ImuCalibrateCommandTest, calibration_is_requested_once_the_turret_is_level_and_still)
{
    command.initialize();
    command.execute();

    EXPECT_EQ(1, imu.calibrationRequests);
    EXPECT_FALSE(command.isFinished());
}

TEST_F(ImuCalibrateCommandTest, calibration_waits_for_a_moving_turret_to_settle)
{
    pitchVelocity = 5.0f;

    command.initialize();
    command.execute();
    EXPECT_EQ(0, imu.calibrationRequests);

    pitchVelocity = 0.0f;
    command.execute();
    EXPECT_EQ(1, imu.calibrationRequests);
}

TEST_F(ImuCalibrateCommandTest, calibration_waits_for_an_off_level_turret_to_reach_level)
{
    pitchPosition = tap::algorithms::Angle(0.5f);

    command.initialize();
    command.execute();
    EXPECT_EQ(0, imu.calibrationRequests);

    pitchPosition = tap::algorithms::Angle(0.0f);
    command.execute();
    EXPECT_EQ(1, imu.calibrationRequests);
}

TEST_F(ImuCalibrateCommandTest, a_turret_that_never_settles_calibrates_anyway_after_the_timeout)
{
    pitchVelocity = 5.0f;

    command.initialize();
    command.execute();
    EXPECT_EQ(0, imu.calibrationRequests);

    clock.time = 5001;
    command.execute();
    EXPECT_EQ(1, imu.calibrationRequests);
}

TEST_F(ImuCalibrateCommandTest, command_finishes_when_the_imu_reports_itself_calibrated)
{
    command.initialize();
    command.execute();
    EXPECT_FALSE(command.isFinished());

    imu.state = ImuInterface::ImuState::IMU_CALIBRATED;
    EXPECT_TRUE(command.isFinished());
}

TEST_F(ImuCalibrateCommandTest, command_gives_up_the_turret_if_calibration_never_completes)
{
    command.initialize();
    command.execute();

    clock.time = 6001;
    EXPECT_TRUE(command.isFinished());
}

TEST_F(ImuCalibrateCommandTest, turret_is_left_stationary_when_the_command_ends)
{
    // Off level, so the command is actively driving the turret when it gets interrupted.
    pitchPosition = tap::algorithms::Angle(0.5f);

    int32_t lastYawOutput = 0;
    int32_t lastPitchOutput = 0;
    ON_CALL(yawMotor, setDesiredOutput).WillByDefault(SaveArg<0>(&lastYawOutput));
    ON_CALL(pitchMotor, setDesiredOutput).WillByDefault(SaveArg<0>(&lastPitchOutput));

    command.initialize();
    command.execute();
    ASSERT_NE(0, lastPitchOutput);

    command.end(true);

    EXPECT_EQ(0, lastYawOutput);
    EXPECT_EQ(0, lastPitchOutput);
}

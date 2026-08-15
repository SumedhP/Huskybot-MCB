#include <gtest/gtest.h>

#include "tap/drivers.hpp"
#include "tap/mock/motor_interface_mock.hpp"

#include "communication/chassis_power_sensors.hpp"
#include "subsystems/chassis/chassis_subsystem.hpp"

using namespace testing;
using namespace huskybot::subsystems::chassis;

static constexpr ChassisGeometry GEOMETRY{.wheelRadius = 0.076f, .lengthX = 0.2f, .lengthY = 0.15f};

class ChassisSubsystemTest : public Test
{
protected:
    ChassisSubsystemTest()
        : voltageSensor(24000.0f),
          powerLimiter(&drivers, &currentSensor, &voltageSensor, 60.0f, 60.0f, 5.0f),
          chassis(
              &drivers,
              motors[LEFT_FRONT],
              motors[RIGHT_FRONT],
              motors[LEFT_BACK],
              motors[RIGHT_BACK],
              mecanumWheelMatrix(GEOMETRY),
              {.kp = 1.0f, .maxOutput = 30000.0f},
              powerLimiter)
    {
    }

    void SetUp() override
    {
        for (int wheel = 0; wheel < NUM_WHEELS; wheel++)
        {
            ON_CALL(*motors[wheel].getEncoder(), getVelocity)
                .WillByDefault(ReturnPointee(&wheelVelocities[wheel]));
            ON_CALL(motors[wheel], isMotorOnline).WillByDefault(ReturnPointee(&motorsOnline));
        }
    }

    tap::Drivers drivers;
    NiceMock<tap::mock::MotorInterfaceMock> motors[NUM_WHEELS];
    huskybot::communication::NominalVoltageSensor voltageSensor;
    huskybot::communication::NoCurrentSensor currentSensor;
    tap::control::chassis::PowerLimiter powerLimiter;
    ChassisSubsystem chassis;

    float wheelVelocities[NUM_WHEELS] = {0.0f, 0.0f, 0.0f, 0.0f};
    bool motorsOnline = true;
};

TEST_F(ChassisSubsystemTest, initialize_initializes_every_motor)
{
    for (int wheel = 0; wheel < NUM_WHEELS; wheel++)
    {
        EXPECT_CALL(motors[wheel], initialize);
    }

    chassis.initialize();
}

TEST_F(ChassisSubsystemTest, desired_velocity_is_reported_back_unchanged)
{
    chassis.setDesiredVelocity({.x = 1.0f, .y = 2.0f, .r = 3.0f});

    EXPECT_FLOAT_EQ(1.0f, chassis.getDesiredVelocity().x);
    EXPECT_FLOAT_EQ(2.0f, chassis.getDesiredVelocity().y);
    EXPECT_FLOAT_EQ(3.0f, chassis.getDesiredVelocity().r);
}

TEST_F(ChassisSubsystemTest, refresh_drives_every_wheel_towards_its_kinematics_speed)
{
    // Driving straight forward asks all four mecanum wheels for the same positive speed, and none
    // of them are moving yet, so all four should be commanded positive.
    chassis.setDesiredVelocity({.x = 1.0f});

    for (int wheel = 0; wheel < NUM_WHEELS; wheel++)
    {
        EXPECT_CALL(motors[wheel], setDesiredOutput(Gt(0)));
    }

    chassis.refresh();
}

TEST_F(ChassisSubsystemTest, refresh_cuts_every_wheel_when_the_power_buffer_is_spent)
{
    tap::communication::serial::RefSerial::Rx::RobotData robotData{};
    robotData.chassis.powerBuffer = 0;
    // Any non-zero timestamp counts as fresh referee data, which snaps the limiter's buffer
    // estimate onto the referee system's own (empty) one.
    robotData.chassis.powerHeatDataReceivedTimestamp = 1;

    ON_CALL(drivers.refSerial, getRefSerialReceivingData).WillByDefault(Return(true));
    ON_CALL(drivers.refSerial, getRobotData).WillByDefault(ReturnRef(robotData));

    chassis.setDesiredVelocity({.x = 1.0f});

    for (int wheel = 0; wheel < NUM_WHEELS; wheel++)
    {
        EXPECT_CALL(motors[wheel], setDesiredOutput(0));
    }

    chassis.refresh();
}

TEST_F(ChassisSubsystemTest, current_velocity_comes_back_out_of_the_measured_wheel_speeds)
{
    // Spin up the wheels exactly as the inverse kinematics would for this velocity.
    const ChassisVelocity commanded{.x = 1.5f, .y = -0.75f, .r = 2.0f};
    tap::algorithms::CMSISMat<NUM_WHEELS, 1> speeds =
        mecanumWheelMatrix(GEOMETRY) *
        tap::algorithms::CMSISMat<3, 1>({commanded.x, commanded.y, commanded.r});

    for (int wheel = 0; wheel < NUM_WHEELS; wheel++)
    {
        wheelVelocities[wheel] = speeds[wheel];
    }

    EXPECT_NEAR(commanded.x, chassis.getCurrentVelocity().x, 1e-3);
    EXPECT_NEAR(commanded.y, chassis.getCurrentVelocity().y, 1e-3);
    EXPECT_NEAR(commanded.r, chassis.getCurrentVelocity().r, 1e-3);
}

TEST_F(ChassisSubsystemTest, refresh_safe_disconnect_zeroes_the_desired_velocity_and_motors)
{
    chassis.setDesiredVelocity({.x = 1.0f, .y = 1.0f, .r = 1.0f});

    for (int wheel = 0; wheel < NUM_WHEELS; wheel++)
    {
        EXPECT_CALL(motors[wheel], setDesiredOutput(0));
    }

    chassis.refreshSafeDisconnect();

    EXPECT_FLOAT_EQ(0.0f, chassis.getDesiredVelocity().x);
    EXPECT_FLOAT_EQ(0.0f, chassis.getDesiredVelocity().y);
    EXPECT_FLOAT_EQ(0.0f, chassis.getDesiredVelocity().r);
}

TEST_F(ChassisSubsystemTest, is_online_only_when_every_motor_is_online)
{
    EXPECT_TRUE(chassis.isOnline());

    motorsOnline = false;
    EXPECT_FALSE(chassis.isOnline());
}

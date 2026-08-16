#include <cmath>

#include <gtest/gtest.h>

#include "tap/drivers.hpp"
#include "tap/mock/abstract_imu_mock.hpp"
#include "tap/mock/motor_interface_mock.hpp"

#include "communication/chassis_power_sensors.hpp"
#include "subsystems/chassis/chassis_beyblade_command.hpp"
#include "subsystems/chassis/chassis_drive_command.hpp"

using namespace testing;
using namespace huskybot::subsystems::chassis;
using namespace huskybot::subsystems::turret;

static constexpr ChassisGeometry GEOMETRY{.wheelRadius = 0.076f, .lengthX = 0.2f, .lengthY = 0.15f};

static constexpr float BEYBLADE_RATE = 2.0f;

/// Operator input with directly settable values, standing in for a real input device.
class TestOperatorInterface : public huskybot::control::ControlOperatorInterface
{
public:
    explicit TestOperatorInterface(tap::Drivers& drivers) : ControlOperatorInterface(drivers, {}) {}

    float getChassisXInput() override { return x; }
    float getChassisYInput() override { return y; }
    float getChassisRotationInput() override { return r; }

    float x = 0.0f;
    float y = 0.0f;
    float r = 0.0f;
};

class ChassisCommandsTest : public Test
{
protected:
    ChassisCommandsTest()
        : turret(&drivers, yawMotor, pitchMotor, imu, {}, {}),
          transforms(turret, {}),
          voltageSensor(24000.0f),
          powerLimiter(&drivers, &currentSensor, &voltageSensor, 60.0f, 60.0f, 5.0f),
          chassis(
              &drivers,
              wheelMotors[LEFT_FRONT],
              wheelMotors[RIGHT_FRONT],
              wheelMotors[LEFT_BACK],
              wheelMotors[RIGHT_BACK],
              omniWheelMatrix(GEOMETRY),
              {.kp = 1.0f, .maxOutput = 30000.0f},
              powerLimiter),
          operatorInterface(drivers),
          driveCommand(chassis, operatorInterface, transforms),
          beybladeCommand(chassis, operatorInterface, transforms, BEYBLADE_RATE)
    {
    }

    void SetUp() override
    {
        ON_CALL(*yawMotor.getEncoder(), getPosition).WillByDefault(ReturnPointee(&turretYaw));
        ON_CALL(*pitchMotor.getEncoder(), getPosition).WillByDefault(ReturnPointee(&turretPitch));

        for (int wheel = 0; wheel < NUM_WHEELS; wheel++)
        {
            ON_CALL(*wheelMotors[wheel].getEncoder(), getVelocity)
                .WillByDefault(ReturnPointee(&wheelVelocities[wheel]));
        }
    }

    /// Spins the wheels as though the chassis were rotating at the given rate.
    void setMeasuredRotation(float rotationRate)
    {
        tap::algorithms::CMSISMat<NUM_WHEELS, 1> speeds =
            omniWheelMatrix(GEOMETRY) * tap::algorithms::CMSISMat<3, 1>({0.0f, 0.0f, rotationRate});

        for (int wheel = 0; wheel < NUM_WHEELS; wheel++)
        {
            wheelVelocities[wheel] = speeds[wheel];
        }
    }

    tap::Drivers drivers;
    NiceMock<tap::mock::MotorInterfaceMock> yawMotor;
    NiceMock<tap::mock::MotorInterfaceMock> pitchMotor;
    NiceMock<tap::mock::AbstractImuMock> imu;
    NiceMock<tap::mock::MotorInterfaceMock> wheelMotors[NUM_WHEELS];

    TurretSubsystem turret;
    huskybot::algorithms::transforms::TransformManager transforms;
    huskybot::communication::NominalVoltageSensor voltageSensor;
    huskybot::communication::NoCurrentSensor currentSensor;
    tap::control::chassis::PowerLimiter powerLimiter;
    ChassisSubsystem chassis;
    TestOperatorInterface operatorInterface;
    ChassisDriveCommand driveCommand;
    ChassisBeybladeCommand beybladeCommand;

    tap::algorithms::WrappedFloat turretYaw = tap::algorithms::Angle(0);
    tap::algorithms::WrappedFloat turretPitch = tap::algorithms::Angle(0);
    float wheelVelocities[NUM_WHEELS] = {0.0f, 0.0f, 0.0f, 0.0f};
};

TEST_F(ChassisCommandsTest, drive_passes_input_straight_through_with_the_turret_centered)
{
    operatorInterface.x = 1.0f;
    operatorInterface.r = 0.5f;

    transforms.update();
    driveCommand.execute();

    EXPECT_NEAR(1.0f, chassis.getDesiredVelocity().x, 1e-5);
    EXPECT_NEAR(0.0f, chassis.getDesiredVelocity().y, 1e-5);
    EXPECT_NEAR(0.5f, chassis.getDesiredVelocity().r, 1e-5);
}

TEST_F(ChassisCommandsTest, drive_rotates_input_out_of_the_turret_frame)
{
    // Turret pointing 90 degrees to the left of the chassis, so "forward" for the operator is the
    // chassis' left.
    turretYaw = tap::algorithms::Angle(M_PI / 2);
    operatorInterface.x = 1.0f;

    transforms.update();
    driveCommand.execute();

    EXPECT_NEAR(0.0f, chassis.getDesiredVelocity().x, 1e-5);
    EXPECT_NEAR(1.0f, chassis.getDesiredVelocity().y, 1e-5);
}

TEST_F(ChassisCommandsTest, drive_end_stops_the_chassis)
{
    operatorInterface.x = 1.0f;
    transforms.update();
    driveCommand.execute();

    driveCommand.end(false);

    EXPECT_FLOAT_EQ(0.0f, chassis.getDesiredVelocity().x);
    EXPECT_FLOAT_EQ(0.0f, chassis.getDesiredVelocity().r);
}

TEST_F(ChassisCommandsTest, beyblade_always_commands_its_target_rotation_rate)
{
    transforms.update();
    beybladeCommand.execute();

    EXPECT_NEAR(BEYBLADE_RATE, chassis.getDesiredVelocity().r, 1e-5);
}

TEST_F(ChassisCommandsTest, beyblade_leaves_translation_alone_once_up_to_speed)
{
    operatorInterface.x = 1.0f;
    setMeasuredRotation(BEYBLADE_RATE);

    transforms.update();
    beybladeCommand.execute();

    EXPECT_NEAR(1.0f, chassis.getDesiredVelocity().x, 1e-3);
}

TEST_F(ChassisCommandsTest, beyblade_decays_translation_in_proportion_to_the_rotation_shortfall)
{
    operatorInterface.x = 1.0f;
    setMeasuredRotation(BEYBLADE_RATE / 4);

    transforms.update();
    beybladeCommand.execute();

    EXPECT_NEAR(0.25f, chassis.getDesiredVelocity().x, 1e-3);
}

TEST_F(ChassisCommandsTest, beyblade_cuts_translation_entirely_when_the_chassis_is_stalled)
{
    operatorInterface.x = 1.0f;
    setMeasuredRotation(0.0f);

    transforms.update();
    beybladeCommand.execute();

    EXPECT_NEAR(0.0f, chassis.getDesiredVelocity().x, 1e-5);
}

TEST_F(ChassisCommandsTest, beyblade_decays_the_turret_relative_translation_not_the_raw_input)
{
    turretYaw = tap::algorithms::Angle(M_PI / 2);
    operatorInterface.x = 1.0f;
    setMeasuredRotation(BEYBLADE_RATE / 2);

    transforms.update();
    beybladeCommand.execute();

    EXPECT_NEAR(0.0f, chassis.getDesiredVelocity().x, 1e-3);
    EXPECT_NEAR(0.5f, chassis.getDesiredVelocity().y, 1e-3);
}

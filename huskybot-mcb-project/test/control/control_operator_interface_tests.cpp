#include <gtest/gtest.h>

#include "tap/architecture/clock.hpp"
#include "tap/drivers.hpp"

#include "control/control_operator_interface.hpp"

using namespace testing;
using huskybot::control::ControlOperatorInterface;
using huskybot::control::ControlOperatorInterfaceConfig;
using tap::communication::serial::Remote;

static constexpr ControlOperatorInterfaceConfig CONFIG = {
    .maxTranslationSpeed = 2.0f,
    .maxRotationSpeed = 4.0f,
    .maxTurretYawSpeed = 6.0f,
    .maxTurretPitchSpeed = 3.0f,
    .mouseYawSensitivity = -0.1f,
    .mousePitchSensitivity = -0.2f,
    .translationAcceleration = 1.0f,
    .rotationAcceleration = 1.0f,
    .sprintMultiplier = 2.0f,
    .slowMultiplier = 0.5f,
    .stickDeadzone = 0.05f,
};

class ControlOperatorInterfaceTest : public Test
{
protected:
    ControlOperatorInterfaceTest() : operatorInterface(drivers, CONFIG) {}

    void SetUp() override
    {
        ON_CALL(drivers.remote, getChannel).WillByDefault(Return(0.0f));
        ON_CALL(drivers.remote, keyPressed).WillByDefault(Return(false));
        ON_CALL(drivers.remote, getMouseX).WillByDefault(Return(0));
        ON_CALL(drivers.remote, getMouseY).WillByDefault(Return(0));
    }

    /// Runs the ramp out far enough that it reaches whatever it was asked for.
    void settleRamps()
    {
        for (int i = 0; i < 10; i++)
        {
            clock.time += 1000;
            operatorInterface.getChassisXInput();
            operatorInterface.getChassisYInput();
            operatorInterface.getChassisRotationInput();
        }
    }

    tap::arch::clock::ClockStub clock;
    tap::Drivers drivers;
    ControlOperatorInterface operatorInterface;
};

TEST_F(ControlOperatorInterfaceTest, chassis_input_scales_full_stick_to_max_speed)
{
    ON_CALL(drivers.remote, getChannel(Remote::Channel::LEFT_VERTICAL)).WillByDefault(Return(1.0f));
    settleRamps();

    EXPECT_NEAR(CONFIG.maxTranslationSpeed, operatorInterface.getChassisXInput(), 1e-5);
    EXPECT_NEAR(0.0f, operatorInterface.getChassisYInput(), 1e-5);
}

TEST_F(ControlOperatorInterfaceTest, chassis_input_sums_stick_and_keyboard_but_clamps_to_max_speed)
{
    ON_CALL(drivers.remote, getChannel(Remote::Channel::LEFT_VERTICAL)).WillByDefault(Return(0.5f));
    ON_CALL(drivers.remote, keyPressed(Remote::Key::W)).WillByDefault(Return(true));
    settleRamps();

    EXPECT_NEAR(CONFIG.maxTranslationSpeed, operatorInterface.getChassisXInput(), 1e-5);
}

TEST_F(ControlOperatorInterfaceTest, keyboard_only_drives_the_chassis)
{
    ON_CALL(drivers.remote, keyPressed(Remote::Key::A)).WillByDefault(Return(true));
    ON_CALL(drivers.remote, keyPressed(Remote::Key::Q)).WillByDefault(Return(true));
    settleRamps();

    EXPECT_NEAR(CONFIG.maxTranslationSpeed, operatorInterface.getChassisYInput(), 1e-5);
    EXPECT_NEAR(CONFIG.maxRotationSpeed, operatorInterface.getChassisRotationInput(), 1e-5);
}

TEST_F(ControlOperatorInterfaceTest, opposing_keys_cancel)
{
    ON_CALL(drivers.remote, keyPressed(Remote::Key::W)).WillByDefault(Return(true));
    ON_CALL(drivers.remote, keyPressed(Remote::Key::S)).WillByDefault(Return(true));
    settleRamps();

    EXPECT_NEAR(0.0f, operatorInterface.getChassisXInput(), 1e-5);
}

TEST_F(ControlOperatorInterfaceTest, chassis_input_ramps_rather_than_stepping)
{
    ON_CALL(drivers.remote, keyPressed(Remote::Key::W)).WillByDefault(Return(true));

    // Prime the ramp's timestamp, then take a quarter second at the configured rate of 1/s.
    operatorInterface.getChassisXInput();
    clock.time += 250;

    EXPECT_NEAR(0.25f * CONFIG.maxTranslationSpeed, operatorInterface.getChassisXInput(), 1e-5);
}

TEST_F(ControlOperatorInterfaceTest, stick_noise_inside_the_deadzone_is_ignored)
{
    ON_CALL(drivers.remote, getChannel(Remote::Channel::LEFT_HORIZONTAL))
        .WillByDefault(Return(0.04f));
    ON_CALL(drivers.remote, getChannel(Remote::Channel::RIGHT_HORIZONTAL))
        .WillByDefault(Return(0.04f));
    settleRamps();

    EXPECT_NEAR(0.0f, operatorInterface.getChassisYInput(), 1e-5);
    EXPECT_NEAR(0.0f, operatorInterface.getTurretYawInput(), 1e-5);
}

TEST_F(ControlOperatorInterfaceTest, shift_sprints_and_ctrl_slows)
{
    ON_CALL(drivers.remote, getChannel(Remote::Channel::LEFT_VERTICAL)).WillByDefault(Return(1.0f));
    ON_CALL(drivers.remote, keyPressed(Remote::Key::SHIFT)).WillByDefault(Return(true));
    settleRamps();

    EXPECT_NEAR(
        CONFIG.sprintMultiplier * CONFIG.maxTranslationSpeed,
        operatorInterface.getChassisXInput(),
        1e-5);

    ON_CALL(drivers.remote, keyPressed(Remote::Key::SHIFT)).WillByDefault(Return(false));
    ON_CALL(drivers.remote, keyPressed(Remote::Key::CTRL)).WillByDefault(Return(true));

    EXPECT_NEAR(
        CONFIG.slowMultiplier * CONFIG.maxTranslationSpeed,
        operatorInterface.getChassisXInput(),
        1e-5);
}

TEST_F(ControlOperatorInterfaceTest, turret_input_sums_stick_and_mouse_without_ramping)
{
    ON_CALL(drivers.remote, getChannel(Remote::Channel::RIGHT_HORIZONTAL))
        .WillByDefault(Return(0.5f));
    ON_CALL(drivers.remote, getChannel(Remote::Channel::RIGHT_VERTICAL))
        .WillByDefault(Return(-0.5f));
    ON_CALL(drivers.remote, getMouseX).WillByDefault(Return(10));
    ON_CALL(drivers.remote, getMouseY).WillByDefault(Return(-10));

    EXPECT_NEAR(
        0.5f * CONFIG.maxTurretYawSpeed + 10 * CONFIG.mouseYawSensitivity,
        operatorInterface.getTurretYawInput(),
        1e-5);
    EXPECT_NEAR(
        -0.5f * CONFIG.maxTurretPitchSpeed - 10 * CONFIG.mousePitchSensitivity,
        operatorInterface.getTurretPitchInput(),
        1e-5);
}

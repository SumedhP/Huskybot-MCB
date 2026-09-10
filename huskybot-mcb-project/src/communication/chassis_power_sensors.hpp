#pragma once

#include "tap/communication/sensors/current/current_sensor_interface.hpp"
#include "tap/communication/sensors/voltage/voltage_sensor_interface.hpp"

namespace huskybot::communication
{
/**
 * Reports a fixed battery voltage.
 */
class DummyVoltageSensor : public tap::communication::sensors::voltage::VoltageSensorInterface
{
public:
    explicit DummyVoltageSensor(float voltageMv) : voltageMv(voltageMv) {}

    float getVoltageMv() const override { return voltageMv; }

    void update() override {}

private:
    float voltageMv;
};

/**
 * Reports zero current draw, for a chassis with no current sensor wired up.
 */
class DummyCurrentSensor : public tap::communication::sensors::current::CurrentSensorInterface
{
public:
    float getCurrentMa() const override { return 0.0f; }

    void update() override {}
};

}  // namespace huskybot::communication

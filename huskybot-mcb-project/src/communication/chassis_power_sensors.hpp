#pragma once

#include "tap/communication/sensors/current/current_sensor_interface.hpp"
#include "tap/communication/sensors/voltage/voltage_sensor_interface.hpp"

namespace huskybot::communication
{
/**
 * Reports a fixed battery voltage. The MCB has no voltage sensor of its own, and the referee
 * system no longer reports chassis voltage, so the battery's nominal voltage is the best estimate
 * available. Good enough because a charged battery only sags a volt or two under load.
 */
class NominalVoltageSensor : public tap::communication::sensors::voltage::VoltageSensorInterface
{
public:
    explicit NominalVoltageSensor(float nominalVoltageMv) : nominalVoltageMv(nominalVoltageMv) {}

    float getVoltageMv() const override { return nominalVoltageMv; }

    void update() override {}

private:
    float nominalVoltageMv;
};

/**
 * Reports zero current draw, for a chassis with no current sensor wired up.
 *
 * `tap::control::chassis::PowerLimiter` uses the current sensor only to integrate power between
 * referee system updates; the buffer is snapped back to the referee system's own number every
 * time one arrives. So with this sensor the limiting still works, it just reacts at the referee
 * system's update rate instead of the control loop's.
 *
 * ponytail: no local current measurement, so brief overdraws between referee packets go
 * uncorrected. Swap in `tap::communication::sensors::current::AnalogCurrentSensor` if a current
 * sensor gets wired to an analog pin.
 */
class NoCurrentSensor : public tap::communication::sensors::current::CurrentSensorInterface
{
public:
    float getCurrentMa() const override { return 0.0f; }

    void update() override {}
};

}  // namespace huskybot::communication

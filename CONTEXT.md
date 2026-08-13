# Huskybot MCB

Firmware for a turreted Type C MCB combat robot, built on taproot. Provides subsystems, commands, and governors for chassis, flywheel, agitator, and turret control, plus supporting algorithms (heat prediction, heat/power limiting).

## Language

**Chassis Frame**:
An angle or velocity measured relative to the robot chassis, derived from a motor encoder (plus a configured mechanical zero offset). Doesn't require the IMU.

**World Frame**:
An angle or velocity measured relative to a fixed external reference, read directly from a turret-mounted IMU (`getYaw()`/`getPitch()`/`getGz()`/`getGy()`). Since the IMU is turret-mounted (this is a turret-side MCB), world frame is a direct passthrough of the IMU's own reading — not a combination with chassis-frame/encoder data.

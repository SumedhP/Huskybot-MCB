# Goal

For 1v1 (but also modular support), a basic system to support a turreted Type C MCB standard robot. Should comply with basic rules, and provide code to assist with heating limit, power limit, and referee-provided speed limit.  Everything should be written minimally in complexity, with guides explaining how to tune the systems and then suggestions on how to further improve it time willing or in the next year. 

# Code attributes

## Subsystems:

- Agitator → Positional control between a series of setpoints. Just goes between the setpoints and says when it gets there. Positional control here because at a higher level, you will be doing shot-timing, which requires good positional control and since you’re heat limited your only doing a few movements at a time. Tuning is straightforward?
- Chassis is Holonomic, ideally easy to support X-drive and Mecanum. Should operate in chassis frame since no chassis imu is assumed to exist. Could subscribe to a chassis heading angle provider? or just work in chassis frame and leave that as an option for teams to do in their own code. tuning is chassis pid. We should provide a drive command and a toggle able beyblade command (targets a rotational speed, higher when static lower when moving).
- flywheel is just two flywheels, simple pid targeting one velocity setpoint.
- Turret is assumed regular turret set up of 2 6020s. always has an imu so either in imu control or in calibration. should give chassis frame and world frame angles, expose something to set motor outputs.

## Components:

- Transform Manager. part of drivers
- HUD Manager? look at rhit design more
- Heat limiter → increment per request to agitator to do prediction and then some buffer limit. updated in drivers
- power limiter → standard tap power limit for chassis
- overspeed protection → just target a lower speed (22 instead of 25)
- control operator interface

HUD can show if various components are online (flywheels on/off being maybe unique?) and then shows the calibration states while imu calibrate
#pragma once

// Selects which robot's control setup `main.cpp` talks to. The TARGET_* macro comes from
// `robot-type/robot_type.hpp` (or `scons robot=TARGET_...`). To add a robot: create
// `robot/<name>/<name>_drivers.hpp` and `<name>_control.cpp`, then add an #elif here and the
// matching one in `drivers_singleton.hpp`/`.cpp`.
#if defined(TARGET_STANDARD)
#include "robot/standard/standard_drivers.hpp"
namespace huskybot::standard
#endif
{
/**
 * Constructs every subsystem and command for this robot, registers them with the command
 * scheduler, and maps them to operator input. Call once, from `main`, after IO is initialized.
 */
void initSubsystemCommands(Drivers *drivers);

/**
 * Per-loop update for robot state that isn't a Subsystem and so doesn't get refreshed by the
 * command scheduler (transforms, heat prediction). Call from `main`'s IO update.
 */
void updateRobotIo(Drivers *drivers);
}  // namespace huskybot::<robot>

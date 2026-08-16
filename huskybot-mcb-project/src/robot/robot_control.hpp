#pragma once

#include "tap/drivers.hpp"

namespace huskybot
{
/**
 * Constructs every subsystem and command for this robot, registers them with the command
 * scheduler, and maps them to operator input. Call once, from `main`, after IO is initialized.
 *
 * Exactly one `robot/<name>/<name>_control.cpp` defines these -- each guards itself on its
 * `TARGET_*` macro, which comes from `robot-type/robot_type.hpp` (or `scons robot=TARGET_...`).
 * To add a robot, add that pair of files; nothing here changes.
 */
void initSubsystemCommands(tap::Drivers &drivers);

/**
 * Robot-specific hardware reads, called as fast as the main loop runs, alongside taproot's own
 * `remote.read()`/`bmi088.read()`.
 */
void updateRobotIo(tap::Drivers &drivers);

/**
 * Per-loop update for robot state that isn't a Subsystem and so doesn't get refreshed by the
 * command scheduler (transforms, heat prediction). Called at scheduler rate, before the scheduler
 * runs, so commands see fresh state.
 */
void updateRobotState(tap::Drivers &drivers);
}  // namespace huskybot

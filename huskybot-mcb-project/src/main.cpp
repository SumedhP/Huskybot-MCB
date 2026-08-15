/*
 * Copyright (c) 2020-2021 huskybot
 *
 * This file is part of huskybot-mcb.
 *
 * huskybot-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * huskybot-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with huskybot-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "tap/board/board.hpp"

#include "modm/architecture/interface/delay.hpp"

/* arch includes ------------------------------------------------------------*/
#include "tap/architecture/periodic_timer.hpp"

/* communication includes ---------------------------------------------------*/
#include "robot/robot_control.hpp"

#include "drivers_singleton.hpp"

/* error handling includes --------------------------------------------------*/
#include "tap/errors/create_errors.hpp"

/* control includes ---------------------------------------------------------*/
#include "tap/architecture/clock.hpp"

using namespace huskybot::standard;

/* define timers here -------------------------------------------------------*/
static constexpr float MAIN_LOOP_FREQUENCY = 1000.0f;
tap::arch::PeriodicMilliTimer sendMotorTimeout(1000.0f / MAIN_LOOP_FREQUENCY);

// Place any sort of input/output initialization here. For example, place
// serial init stuff here.
static void initializeIo(Drivers* drivers);

// Anything that you would like to be called place here. It will be called
// very frequently. Use PeriodicMilliTimers if you don't want something to be
// called as frequently.
static void updateIo(Drivers* drivers);

int main()
{
    /*
     * NOTE: We are using DoNotUse_getDrivers here because in the main
     *      robot loop we must access the singleton drivers to update
     *      IO states and run the scheduler.
     */
    Drivers* drivers = DoNotUse_getDrivers();

    Board::initialize();
    initializeIo(drivers);
    initSubsystemCommands(drivers);

    while (1)
    {
        // do this as fast as you can
        updateIo(drivers);

        if (sendMotorTimeout.execute())
        {
            drivers->bmi088.periodicIMUUpdate();
            drivers->commandScheduler.run();
            drivers->djiMotorTxHandler.encodeAndSendCanData();
        }
        modm::delay_us(10);
    }
    return 0;
}

static void initializeIo(Drivers* drivers)
{
    drivers->pwm.init();
    drivers->digital.init();
    drivers->leds.init();
    drivers->can.initialize();
    drivers->remote.initialize();

    drivers->bmi088.initialize(MAIN_LOOP_FREQUENCY, 0.1, 0);
    drivers->bmi088.setTargetTemperature(35.0f);
    drivers->bmi088.setCalibrationSamples(4000);

    drivers->refSerial.initialize();
}

static void updateIo(Drivers* drivers)
{
    drivers->canRxHandler.pollCanData();
    drivers->refSerial.updateSerial();
    drivers->remote.read();
    drivers->bmi088.read();
    updateRobotIo(drivers);
}

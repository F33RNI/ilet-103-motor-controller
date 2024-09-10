/**
 * @file rotarySwitch.cpp
 * @author Fern Lane
 * @brief Reads RPM setpoint from rotary switch
 *
 * @copyright Copyright (c) 2024 Fern Lane
 *
 * This file is part of the ilet-103-motor-controller distribution.
 * See <https://github.com/F33RNI/ilet-103-motor-controller> for more info.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * long with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <Arduino.h>

#include "include/config.h"
#include "include/pins.h"

#include "include/rotary_switch.h"

// Preinstantiate
RotarySwitch rotarySwitch;

/**
 * @brief Constructs a new Rotary Switch:: Rotary Switch object
 */
RotarySwitch::RotarySwitch(void) {
    rpm_setpoint_raw = RPM_SETPOINTS[0];
    rpm_setpoint_filtered = RPM_SETPOINTS[0];
}

/**
 * @brief Reads states of rotary switch without filtering it
 * Call filter_setpoint() to filter it and get filtered value
 *
 * @return float unfiltered (RAW) RPM
 */
float RotarySwitch::read_setpoint(void) {
    for (uint8_t i = 0; i < sizeof(SPEED_SELECTOR_PINS) / sizeof(uint8_t); ++i)
        if (!digitalRead(SPEED_SELECTOR_PINS[i]))
            rpm_setpoint_raw = RPM_SETPOINTS[i];

    return rpm_setpoint_raw;
}

/**
 * @brief Filters setpoint. This must be called with normal cycle time for proper filtering
 *
 * @return float filtered RPM
 */
float RotarySwitch::filter_setpoint(void) {
    rpm_setpoint_filtered = rpm_setpoint_filtered * SETPOINT_FILTER + rpm_setpoint_raw * (1.f - SETPOINT_FILTER);
    return rpm_setpoint_filtered;
}

/**
 * @brief Sets rpm_setpoint_raw and rpm_setpoint_filtered to 0
 */
void RotarySwitch::reset(void) {
    rpm_setpoint_raw = 0.f;
    rpm_setpoint_filtered = 0.f;
}

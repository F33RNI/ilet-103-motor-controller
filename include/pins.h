/**
 * @file pins.h
 * @author Fern Lane
 * @brief Digital and analog pins config
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
#ifndef PINS_H__
#define PINS_H__

#include <Arduino.h>

// Rotary switch pins
const uint8_t SPEED_SELECTOR_PINS[] = {4U, 5U, 6U, 7U, 8U};

// Must be 2 or 3
const uint8_t ENCODER_PIN = 3U;

// Must be 9 or 10
const uint8_t MOTOR_PWM_PIN = 9U;

// Comment if LOW=Off, HIGH=On; Uncomment if LOW=On, HIGH=Off
#define MOTOR_PWM_INVERTED

// Can be any pin
const uint8_t COMMAND_19_PIN = 10U;

// Comment if LOW=Off, HIGH=On; Uncomment if LOW=On, HIGH=Off
// #define COMMAND_19_INVERTED

// Can be any pin
const uint8_t MOTOR_ENABLE_PIN = 11U;

// Comment if LOW=Disabled, HIGH=Enabled; Uncomment if LOW=Enabled, HIGH=Disabled
#define MOTOR_ENABLED_INVERTED

#endif

/**
 * @file config.h
 * @author Fern Lane
 * @brief PWM, PID-controller and other configs
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

#ifndef CONFIG_H__
#define CONFIG_H__

#include <Arduino.h>

// Uncomment to receive rpm_setpoint_filtered and rpm_filtered separated by tab (/t) at 115200
#define DEBUG

// In microseconds (1 / (21us / 1000 / 1000) ~= 47619Hz)
const uint64_t MOTOR_PWM_PERIOD = 21UL;

// Encoder ticks per revolution
const float ENCODER_DIVIDER = 4.f;

// Consider RPM as 0 if it's below this value
const float ENCODER_MIN_RPM = 100.f;

// Main low-pass filter. Closer to 1 = smoother and slower
// Actual filter value will be RPM_FILTER / time_delta (in seconds)
// RPM_FILTER = K * (1 / (RPS * ENCODER_DIVIDER))
const float RPM_FILTER = .85f * (1.f / ((1443.76f / 60.f) * ENCODER_DIVIDER));

// Low-pass filter for smoothing setpoint change. Closer to 1 = smoother and slower
const float SETPOINT_FILTER = .94;

// Rotary switch setpoints. Must be the same size as SPEED_SELECTOR_PINS
// RPM @ 6mm: (IPS * 25.4 / (pi * 6 mm)) * 60, RPM @ 100 mm / 42.5 mm: (IPS * 25.4 / (pi * 6 mm)) * 60 * (100 / 42)
// <https://en.wikipedia.org/wiki/Audio_tape_specifications>
// Standard IPS: 1.875, 3.75, 7.5 -> motor RPMs: 360.94, 721.88, 1443.76
const float RPM_SETPOINTS[] = {360.94f, 541.41f, 721.88f, 1082.82f, 1443.76f};

// PID values
const float PID_P_GAIN = .0012f;
const float PID_I_GAIN = .0036f;
const float PID_D_GAIN = .000002f;

// Integral part of the PID will be clamped to this range to prevent integral windup
const float PID_MIN_INTEGRAL = -1000.f;
const float PID_MAX_INTEGRAL = 1000.f;

// Output of the PID will be clamped to this range
const float PID_MIN_OUT = 0.f;
const float PID_MAX_OUT = .8f;

#endif

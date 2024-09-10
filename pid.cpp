/**
 * @file pid.cpp
 * @author Fern Lane
 * @brief PID-controller functions
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

#include "include/pid.h"

/**
 * @brief Constructs a new PID::PID object
 * (wrapper for reset() that initializes class variables)
 */
PID::PID(void) { reset(); }

/**
 * @brief Calculate one cycle of PID-controller
 *
 * @param error setpoint - input
 * @param time_delta current time - previous time (in seconds)
 * @param pid_output existing variable to write into (will be in [-PID_MIN_MAX_OUT, PID_MIN_MAX_OUT] range)
 */
float PID::calculate(float error, float time_delta) {
    // Calculate P term
    float p_output = PID_P_GAIN * error;

    // Calculate I term
    integral_accumulator += error * time_delta;

    // Prevent integral windup
    if (integral_accumulator > PID_MAX_INTEGRAL)
        integral_accumulator = PID_MAX_INTEGRAL;
    else if (integral_accumulator < PID_MIN_INTEGRAL)
        integral_accumulator = PID_MIN_INTEGRAL;

    // Calculate D term
    float d_output = 0.f;
    if (time_delta != 0.f)
        d_output = PID_D_GAIN * ((error - error_prev) / time_delta);
    error_prev = error;

    // Calculate total output
    float pid_output = p_output + PID_I_GAIN * integral_accumulator + d_output;

    // Clamp output
    if (pid_output > PID_MAX_OUT)
        pid_output = PID_MAX_OUT;
    else if (pid_output < PID_MIN_OUT)
        pid_output = PID_MIN_OUT;

    return pid_output;
}

/**
 * @brief Resets PID's I and D variables
 */
void PID::reset(void) {
    integral_accumulator = 0.f;
    error_prev = 0.f;
}

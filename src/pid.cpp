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

#include "config.h"
#include "functions.h"
#include "pid.h"

/**
 * @brief Calculates one cycle of PID-controller
 *
 * @param error setpoint - input
 * @param time_delta current time - previous time (in seconds)
 * @param pid_output pointer to float pid output (will be in [-PID_MIN_MAX_OUT, PID_MIN_MAX_OUT] range)
 */
void pid_calculate(float error, float time_delta, float *pid_output) {
    // Calculate P term
    float pid_p_output = PID_P_GAIN * error;

    // Calculate I term
    pid_i_mem += error * time_delta;

    // Prevent integral windup
    if (pid_i_mem > PID_MAX_INTEGRAL)
        pid_i_mem = PID_MAX_INTEGRAL;
    else if (pid_i_mem < PID_MIN_INTEGRAL)
        pid_i_mem = PID_MIN_INTEGRAL;

    // Calculate D term
    float pid_d_output = 0.f;
    if (time_delta != 0.f)
        pid_d_output = PID_D_GAIN * ((error - pid_prev_d_error) / time_delta);

    // Calculate total output
    *pid_output = pid_p_output + PID_I_GAIN * pid_i_mem + pid_d_output;

    // Clamp output
    if (*pid_output > PID_MAX_OUT)
        *pid_output = PID_MAX_OUT;
    else if (*pid_output < PID_MIN_OUT)
        *pid_output = PID_MIN_OUT;
}

/**
 * @brief Resets PID's I and D variables
 */
void pid_reset(void) {
    pid_i_mem = 0.f;
    pid_prev_d_error = 0.f;
}

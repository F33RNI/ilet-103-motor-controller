/**
 * @file pins.h
 * @author Fern Lane
 * @brief All global function definitions
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

#ifndef FUNCTIONS_H__
#define FUNCTIONS_H__

#include <Arduino.h>

// encoder.cpp
void encoder_init(void);
boolean encoder_stall_check(void);
void encoder_stop(void);
void encoder_resume(void);
float encoder_get_time_delta(void);
float encoder_get_rpm_filtered(void);
boolean encoder_get_triggered(void);
void encoder_clear_triggered(void);

// motor.cpp
void motor_init(void);
void motor_write(float pwm);

// pid.cpp
void pid_calculate(float error, float time_delta, float *pid_output);
void pid_reset(void);

#endif

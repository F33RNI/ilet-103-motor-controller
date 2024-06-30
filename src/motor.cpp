/**
 * @file motor.cpp
 * @author Fern Lane
 * @brief Motor PWM wrapper
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

#include <TimerOne.h>

#include "config.h"
#include "functions.h"
#include "pins.h"

/**
 * @brief Sets up timer1-based pwm (9 and 10 pins on ATmega328)
 */
void motor_init(void) {
    Timer1.initialize(MOTOR_PWM_PERIOD);
#ifdef MOTOR_PWM_INVERTED
    Timer1.pwm(MOTOR_PWM_PIN, 1023U);
#else
    Timer1.pwm(MOTOR_PWM_PIN, 0U);
#endif
}

/**
 * @brief Writes PWM to the motor
 *
 * @param pwm 0 to 1 (0 - off, 1 - on)
 */
void motor_write(float pwm) {
#ifdef MOTOR_PWM_INVERTED
    Timer1.setPwmDuty(MOTOR_PWM_PIN, 1023U - ((uint32_t) (pwm * 1023.f)));
#else
    Timer1.setPwmDuty(MOTOR_PWM_PIN, (uint32_t) (pwm * 1023.f));
#endif
}

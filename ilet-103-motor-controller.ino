/**
 * @file main.cpp
 * @author Fern Lane
 * @brief Digital speed controller for ILET 103 reel-to-reel deck
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

// For Arduino IDE compatibility
#include "include/config.h"
#include "include/pins.h"

// For Arduino IDE compatibility
#include "include/encoder.h"
#include "include/motor.h"
#include "include/pid.h"
#include "include/rotarySwitch.h"

boolean motor_enabled_prev;

PID pid = PID ();

void
setup ()
{
    // Setup encoder
    encoder.init ();

    // Setup rotary switch as pullup pins
    for (uint8_t i = 0; i < sizeof (SPEED_SELECTOR_PINS) / sizeof (uint8_t); ++i)
        pinMode (SPEED_SELECTOR_PINS[i], INPUT_PULLUP);

    // Setup enable
    pinMode (MOTOR_ENABLE_PIN, INPUT_PULLUP);

    // Setup command 19
    pinMode (COMMAND_19_PIN, OUTPUT);
#ifdef COMMAND_19_INVERTED
    digitalWrite (COMMAND_19_PIN, HIGH);
#endif

    // Setup motor
    Motor::init ();

    // Setup serial port for debugging
#ifdef DEBUG
    Serial.begin (115200);
#endif

    // Wait a bit to make sure everything is settled
    delay (100);
}

void
loop ()
{
    // Read rotary switch
    float rpm_setpoint_raw = rotarySwitch.read_setpoint ();

    // Enable / disable command 19 according to RPM
    boolean command_19 = rpm_setpoint_raw > RPM_SETPOINTS[sizeof (RPM_SETPOINTS) / sizeof (float) / 2];
#ifdef COMMAND_19_INVERTED
    digitalWrite (COMMAND_19_PIN, !command_19);
#else
    digitalWrite (COMMAND_19_PIN, command_19);
#endif

    // Check if we're stalled
    boolean is_stalled = encoder.stall_check ();

// Check if we need to enable or disable motor
#ifdef MOTOR_ENABLED_INVERTED
    boolean motor_enabled = !digitalRead (MOTOR_ENABLE_PIN);
#else
    boolean motor_enabled = digitalRead (MOTOR_ENABLE_PIN);
#endif

    // Something changed -> reset setpoint and PID
    if (motor_enabled != motor_enabled_prev)
        {
            rotarySwitch.reset ();
            pid.reset ();
            Motor::write (0.f);

            // Stop or resume encoder
            if (motor_enabled)
                encoder.resume ();
            else
                encoder.stop ();
        }

    // Save for the next cycle
    motor_enabled_prev = motor_enabled;

    // We don't need other encoder and pid actions now
    if (!motor_enabled)
        return;

    // If we have encoder callback or stalled
    if (encoder.get_triggered () || is_stalled)
        {
            // Clear triggered flag
            encoder.clear_triggered ();

            // Filter setpoint
            float rpm_setpoint_filtered = rotarySwitch.filter_setpoint ();

            // Calculate PID only if motor is enabled
            if (motor_enabled)
                {
                    // Calculate PID input error as difference between setpoint and measured RPM
                    float pid_error = rpm_setpoint_filtered - encoder.get_rpm_filtered ();

                    // Calculate PID and it to the motor
                    Motor::write (pid.calculate (pid_error, encoder.get_time_delta ()));
                }

            // Reset PID if motor is disabled
            else
                pid.reset ();

// Plot debug info (only if motor is spinning or stalled)
#ifdef DEBUG
            Serial.print (rpm_setpoint_filtered);
            Serial.print ("\t");
            Serial.println (encoder.get_rpm_filtered ());
#endif
        }

    // Simulate encoder cycle if stalled
    if (is_stalled)
        {
            uint64_t period_min = (1.f / (ENCODER_MIN_RPM / 60.f * ENCODER_DIVIDER)) * 1.e6;
            delay (period_min / 1000);
            delayMicroseconds (period_min % 1000);
        }
}

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

#include "config.h"
#include "functions.h"
#include "pins.h"

float pid_error, pid_output;
float rpm_setpoint_raw, rpm_setpoint_filtered;
boolean motor_enabled_prev;
void read_setpoint(void);

void setup() {
    // Setup encoder
    encoder_init();

    // Setup rotary switch as pullup pins
    for (uint8_t i = 0; i < sizeof(SPEED_SELECTOR_PINS) / sizeof(uint8_t); ++i)
        pinMode(SPEED_SELECTOR_PINS[i], INPUT_PULLUP);
    rpm_setpoint_raw = RPM_SETPOINTS[0];
    rpm_setpoint_filtered = RPM_SETPOINTS[0];

    // Setup enable
    pinMode(MOTOR_ENABLE_PIN, INPUT_PULLUP);

    // Setup command 19
    pinMode(COMMAND_19_PIN, OUTPUT);
#ifdef COMMAND_19_INVERTED
    digitalWrite(COMMAND_19_PIN, HIGH);
#endif

    // Setup motor
    motor_init();

    // Setup serial port for debugging
#ifdef DEBUG
    Serial.begin(115200);
#endif

    // Wait a bit to make sure everything is settled
    delay(100);
}

void loop() {
    // Check if we're stalled
    boolean is_stalled = encoder_stall_check();

// Check if we need to enable or disable motor
#ifdef MOTOR_ENABLED_INVERTED
    boolean motor_enabled = !digitalRead(MOTOR_ENABLE_PIN);
#else
    boolean motor_enabled = digitalRead(MOTOR_ENABLE_PIN);
#endif

    // Something changed -> reset setpoint and PID
    if (motor_enabled != motor_enabled_prev) {
        rpm_setpoint_filtered = 0.f;
        pid_reset();
        motor_write(0.f);

        // Stop or resume encoder
        if (motor_enabled)
            encoder_resume();
        else
            encoder_stop();
    }

    // Save for the next cycle
    motor_enabled_prev = motor_enabled;

    // We don't need other encoder and pid actions now
    if (!motor_enabled)
        return;

    // If we have encoder callback or stalled
    if (encoder_get_triggered() || is_stalled) {
        // Clear triggered flag
        encoder_clear_triggered();

        // Read rotary switch
        read_setpoint();

        // Enable command 19 according to RPM
        boolean command_19 = rpm_setpoint_raw > RPM_SETPOINTS[sizeof(RPM_SETPOINTS) / sizeof(float) / 2];
#ifdef COMMAND_19_INVERTED
        digitalWrite(COMMAND_19_PIN, !command_19);
#else
        digitalWrite(COMMAND_19_PIN, command_19);
#endif

        // Calculate PID only if motor is enabled
        if (motor_enabled) {

            // Calculate PID input error as difference between setpoint and measured RPM
            pid_error = rpm_setpoint_filtered - encoder_get_rpm_filtered();

            // Calculate PID and it to the motor
            pid_calculate(pid_error, encoder_get_time_delta(), &pid_output);
            motor_write(pid_output);
        }

        // Reset PID if motor is disabled
        else
            pid_reset();

// Plot debug info (only if motor is spinning or stalled)
#ifdef DEBUG
        Serial.print(rpm_setpoint_filtered);
        Serial.print("\t");
        Serial.print(encoder_get_rpm_filtered());
        Serial.print("\t");
        Serial.println(pid_output);
#endif
    }

    // Simulate encoder cycle if stalled
    if (is_stalled) {
        uint64_t period_min = (1.f / (ENCODER_MIN_RPM / 60.f * ENCODER_DIVIDER)) * 1.e6;
        delay(period_min / 1000);
        delayMicroseconds(period_min % 1000);
    }
}

/**
 * @brief Reads states of rotary switch and sets rpm_setpoint_filtered value
 */
void read_setpoint(void) {
    // Read unfiltered setpoint
    for (uint8_t i = 0; i < sizeof(SPEED_SELECTOR_PINS) / sizeof(uint8_t); ++i)
        if (!digitalRead(SPEED_SELECTOR_PINS[i]))
            rpm_setpoint_raw = RPM_SETPOINTS[i];

    // Filter it
    rpm_setpoint_filtered = rpm_setpoint_filtered * SETPOINT_FILTER + rpm_setpoint_raw * (1.f - SETPOINT_FILTER);
}

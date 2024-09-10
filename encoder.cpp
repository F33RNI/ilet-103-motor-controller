/**
 * @file encoder.cpp
 * @author Fern Lane
 * @brief Encoder RPM meter
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
#include <util/atomic.h>

#include "include/config.h"
#include "include/pins.h"

#include "include/encoder.h"

// Preinstantiate
Encoder encoder;

/**
 * @brief Sets up encoder pin and connects interrupt callback
 */
void Encoder::init(void) {
    pinMode(ENCODER_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), isr, ENCODER_INTERRUPT_MODE);
    is_interrupt_attached = true;
}

/**
 * @brief Interrupt callback (encoder)
 * Calculates motor speed in RPM. Call check_triggered() to check for new cycle
 */
void Encoder::handle_interrupt(void) {
    uint64_t time_ = micros();

    // micros() overflow
    if (time_prev == 0 || time_prev >= time_) {
        time_prev = time_;
        return;
    }

    // Calculate time between callbacks (encoder period) in seconds
    time_delta = (float) (time_ - time_prev) / 1.e6f;
    time_prev = time_;

    // Calculate unfiltered rpm
    float rpm_raw = (1.f / time_delta) / ENCODER_DIVIDER * 60.f;

    // First time -> don't filter
    if (rpm_filtered == 0.f)
        rpm_filtered = rpm_raw;
    else {
        // Calculate filter with variable cycle time
        float filter_k = RPM_FILTER / time_delta;
        if (filter_k < 0.f)
            filter_k = 0.f;
        else if (filter_k > FILTER_MAX)
            filter_k = FILTER_MAX;

        // Filter it
        rpm_filtered =
            rpm_filtered * filter_k + rpm_raw * (1.f - filter_k) / 2.f + rpm_raw_prev * (1.f - filter_k) / 2.f;
    }
    rpm_raw_prev = rpm_raw;

    // Set triggered flag
    triggered = true;
}

/**
 * @brief Checks if no interrupts in a long time and slowly brings rpm_filtered to 0
 * and time_delta to the actual time delta
 * Call it in a main loop() before everything (before calling clear_triggered())
 *
 * @return boolean true if stalled
 */
boolean Encoder::stall_check(void) {
    uint64_t time_current = micros();

    float time_prev_;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { time_prev_ = time_prev; }

    // Ignore if interrupt is set or time is incorrect
    if (get_triggered() || time_prev_ > time_current) {
        // Fix time
        if (time_prev_ > time_current) {
            noInterrupts();
            time_prev = time_current;
            interrupts();
        }

        return false;
    }

    boolean stalled = false;

    float time_delta_test = (float) (time_current - time_prev_) / 1.e6f;
    if ((1.f / time_delta_test) / ENCODER_DIVIDER * 60.f < ENCODER_MIN_RPM) {

        // Disable interrupts to prevent errors
        noInterrupts();

        // Set variables
        rpm_filtered *= .98f;
        time_delta = time_delta_test;
        stalled = true;

        // Enable interrupts back
        interrupts();
    }

    return stalled;
}

/**
 * @brief Detaches interrupts and sets time_prev to current micros() and time_delta and rpm_filtered to 0
 */
void Encoder::stop(void) {
    // Detach encoder interrupts
    if (is_interrupt_attached) {
        noInterrupts();
        detachInterrupt(digitalPinToInterrupt(ENCODER_PIN));
        is_interrupt_attached = false;
        interrupts();
    }

    time_prev = micros();
    time_delta = 0.f;
    rpm_filtered = 0.f;
}

/**
 * @brief Attaches encoder interrupt and sets time_prev to current micros() and time_delta and rpm_filtered to 0
 */
void Encoder::resume(void) {
    // Attach back
    if (!is_interrupt_attached) {
        attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), isr, ENCODER_INTERRUPT_MODE);
        is_interrupt_attached = true;
    }

    time_prev = micros();
    time_delta = 0.f;
    rpm_filtered = 0.f;
}

/**
 * @return float last cycle time (in seconds)
 */
float Encoder::get_time_delta(void) {
    float time_delta_;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { time_delta_ = time_delta; }
    return time_delta_;
}

/**
 * @return float current RPM (filtered)
 */
float Encoder::get_rpm_filtered(void) {
    float rpm_filtered_;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { rpm_filtered_ = rpm_filtered; }
    return rpm_filtered_;
}

/**
 * @return boolean true if encoder triggered
 * Call clear_triggered() to reset it
 */
boolean Encoder::get_triggered(void) {
    boolean triggered_;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { triggered_ = triggered; }
    return triggered_;
}

/**
 * @brief Sets triggered to false (clears triggered flag)
 */
void Encoder::clear_triggered(void) {
    // Ignore if not triggered to prevent errors
    if (!get_triggered())
        return;
    triggered = false;
}

/**
 * @brief Static interrupt callback (wrapper for handle_interrupt())
 */
void Encoder::isr(void) { encoder.handle_interrupt(); }

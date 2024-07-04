/**
 * @file encoder.h
 * @author Fern Lane
 * @brief Encoder RPM meter variables and definitions
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

#ifndef ENCODER_H__
#define ENCODER_H__

#include <Arduino.h>

#define ENCODER_INTERRUPT_MODE RISING

// Actual filter maximum
const float FILTER_MAX PROGMEM = .990f;

class Encoder
{
  public:
    void init (void);
    boolean stall_check (void);
    void stop (void);
    void resume (void);
    float get_time_delta (void);
    float get_rpm_filtered (void);
    boolean get_triggered (void);
    void clear_triggered (void);

  private:
    volatile uint64_t time_prev;
    volatile float time_delta;
    float rpm_raw_prev;
    volatile float rpm_filtered;
    volatile boolean triggered;
    boolean is_interrupt_attached;

    void handle_interrupt (void);
    static void isr (void);
};

extern Encoder encoder;

#endif

/**
 * @file rotary_switch.h
 * @author Fern Lane
 * @brief Rotary switch class definitions
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

#ifndef ROTARY_SWITCH_H__
#define ROTARY_SWITCH_H__

#include <Arduino.h>

class RotarySwitch {
  private:
    float rpm_setpoint_raw, rpm_setpoint_filtered;

  public:
    RotarySwitch();
    float read_setpoint(void);
    float filter_setpoint(void);
    void reset(void);
};

extern RotarySwitch rotarySwitch;

#endif

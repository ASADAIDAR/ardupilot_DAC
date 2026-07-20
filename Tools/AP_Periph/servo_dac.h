/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <AP_HAL/AP_HAL.h>

/*
 * ServoDACHandler
 * 
 * Handles servo PWM output and DAC voltage output for CAN peripheral boards.
 * Maps PWM commands (1000-2000 us) to DAC voltage (0-3.3V) linearly:
 * - 1000 us = 0.0 V
 * - 1500 us = 1.65 V (center)
 * - 2000 us = 3.3 V
 */
class ServoDACHandler {
public:
    ServoDACHandler();
    
    // Initialize servo DAC handler
    // pwm_channel: PWM output channel (0-based, corresponding to PA0, PA1, etc.)
    // dac_channel: DAC output channel (0=PA4/DAC_CH1, 1=PA5/DAC_CH2)
    void init(uint8_t pwm_channel, uint8_t dac_channel);
    
    // Set servo position from CAN command
    // command_us: servo position in microseconds (typical range 1000-2000)
    void set_servo_position(uint16_t command_us);
    
    // Update PWM and DAC outputs (call regularly from main loop)
    void update();
    
    // Get current servo position in microseconds
    uint16_t get_current_position_us() const { return _current_command_us; }
    
private:
    uint8_t _pwm_channel;           // PWM output channel (0-10)
    uint8_t _dac_channel;           // DAC channel (0 or 1)
    uint16_t _current_command_us;   // Current servo command in microseconds
    uint32_t _last_update_ms;       // Timestamp of last command received
    
    // Conversion constants
    static constexpr uint16_t PWM_MIN_US = 1000;   // Minimum PWM = 0V
    static constexpr uint16_t PWM_MAX_US = 2000;   // Maximum PWM = 3.3V
    static constexpr float DAC_MIN_V = 0.0f;       // Minimum DAC voltage
    static constexpr float DAC_MAX_V = 3.3f;       // Maximum DAC voltage
    static constexpr uint32_t COMMAND_TIMEOUT_MS = 1000;  // Command timeout (1 second)
    
    // Convert PWM value to DAC voltage
    float pwm_to_dac_voltage(uint16_t pwm_us) const;
};

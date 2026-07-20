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

#include "servo_dac.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_DAC/AP_DAC.h>

extern const AP_HAL::HAL &hal;

ServoDACHandler::ServoDACHandler() :
    _pwm_channel(0),
    _dac_channel(0),
    _current_command_us(1500),  // Default to center position
    _last_update_ms(0)
{
}

void ServoDACHandler::init(uint8_t pwm_channel, uint8_t dac_channel)
{
    _pwm_channel = pwm_channel;
    _dac_channel = dac_channel;
    _current_command_us = 1500;  // Default to center position (1500us = 1.65V)
    _last_update_ms = AP_HAL::millis();
}

void ServoDACHandler::set_servo_position(uint16_t command_us)
{
    // Clamp the command value to valid servo range
    _current_command_us = constrain_int16(command_us, PWM_MIN_US, PWM_MAX_US);
    _last_update_ms = AP_HAL::millis();
}

float ServoDACHandler::pwm_to_dac_voltage(uint16_t pwm_us) const
{
    // Linear mapping: 1000us -> 0V, 2000us -> 3.3V
    // Normalize PWM value to 0.0 - 1.0 range
    float normalized = float(pwm_us - PWM_MIN_US) / float(PWM_MAX_US - PWM_MIN_US);
    
    // Ensure normalized value stays within 0.0 - 1.0
    normalized = constrain_float(normalized, 0.0f, 1.0f);
    
    // Convert to voltage
    float voltage = DAC_MIN_V + (normalized * (DAC_MAX_V - DAC_MIN_V));
    
    return voltage;
}

void ServoDACHandler::update()
{
    uint32_t now_ms = AP_HAL::millis();
    
    // Check if command has timed out
    // If no new command received for COMMAND_TIMEOUT_MS, return to safe position
    if (now_ms - _last_update_ms > COMMAND_TIMEOUT_MS) {
        _current_command_us = 1500;  // Safe center position
    }
    
    // Output PWM signal
    hal.rcout->write(_pwm_channel, _current_command_us);
    
    // Calculate and output DAC voltage
    float dac_voltage = pwm_to_dac_voltage(_current_command_us);
    
#if AP_DAC_ENABLED
    AP_DAC* dac = AP::dac();
    if (dac) {
        // Convert voltage to DAC value
        // DAC uses 12-bit: 0-4095 represents 0-3.3V
        // Formula: dac_value = (voltage / 3.3) * 4095
        uint16_t dac_value = (uint16_t)((dac_voltage / 3.3f) * 4095.0f);
        
        // Write value to DAC channel
        dac->write_channel(_dac_channel, dac_value);
    }
#endif
}

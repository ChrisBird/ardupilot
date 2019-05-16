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

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Frsky_Sensor/AP_Frsky_Sensor.h>
#include "AP_BattMonitor_Frsky_Sensor.h"

extern const AP_HAL::HAL &hal;

void AP_BattMonitor_Frsky_Sensor::init(void)
{
    last_called_ms = 0;

    AP_Frsky_Sensor *frskySensor = AP_Frsky_Sensor::get_singleton();
    if (!frskySensor) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "BM: Frsky Read - No Singleton");
        return;
    }
    gcs().send_text(MAV_SEVERITY_WARNING,"BM: FrSky loading");
    frskySensor->init();
}

void AP_BattMonitor_Frsky_Sensor::read(void)
{
    uint32_t now = AP_HAL::millis();
    if (now < (last_called_ms + 1000)) {
        return;
    }
    last_called_ms = now;

    //gcs().send_text(MAV_SEVERITY_CRITICAL, "BM: Frsky Read");
    AP_Frsky_Sensor *frskySensor = AP_Frsky_Sensor::get_singleton();
    if (!frskySensor) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "BM: Frsky Read - No Singleton");
        return;
    }
    //gcs().send_text(MAV_SEVERITY_CRITICAL, "BM: Frsky Read - Got Singleton");

    _state.voltage = frskySensor->mlvss_Voltage_Sum() * 0.001;
    gcs().send_text(MAV_SEVERITY_CRITICAL, "BM: Frsky Read: %.3f", _state.voltage);

    float current_sum = 0;

    if (current_sum > 0) {
        // if we have ever got a current value then we know we have a
        // current sensor
        have_current = true;
    }
}

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

#include "AP_Frsky_Sensor.h"
//#include <AP_Frsky_Telem/AP_Frsky_Telem.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_SerialManager/AP_SerialManager.h>

extern const AP_HAL::HAL& hal;

AP_Frsky_Sensor *AP_Frsky_Sensor::_singleton;

AP_Frsky_Sensor::AP_Frsky_Sensor()
{
    _singleton = this;
    gcs().send_text(MAV_SEVERITY_INFO, "FR Sensor: Init");
    init();
}

void AP_Frsky_Sensor::init()
{
    const AP_SerialManager &serial_manager = AP::serialmanager();

    gcs().send_text(MAV_SEVERITY_INFO, "FR: Finding Serial Devices");
    _port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_FrSky_SPort_Sensor,0);
    gcs().send_text(MAV_SEVERITY_INFO, "FR: Trying to add tick callback");

    if (_port != nullptr) {
        gcs().send_text(MAV_SEVERITY_INFO, "FR: Adding tick callback");
         hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&AP_Frsky_Sensor::tick, void));
         // we don't want flow control
         _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
     }

    mlvss.cell2 = 4000;
}


void AP_Frsky_Sensor::send_byte(uint8_t byte)
{
    //if (byte == START_STOP_SPORT) {
    //    _port->write(0x7D);
    //    _port->write(0x5E);
    //} else {
    _port->write(byte);
    //}
    //calc_crc(byte);
}

void AP_Frsky_Sensor::get(uint8_t id)
{
    //send_byte(0x10); // DATA_FRAME
    send_byte(START_STOP_SPORT);
    uint8_t *bytes = (uint8_t*)&id;
    send_byte(bytes[0]); // LSB
    //send_byte(bytes[1]); // MSB
    //bytes = (uint8_t*)&data;
    //send_byte(bytes[0]); // LSB
    //send_byte(bytes[1]);
    //send_byte(bytes[2]);
    //send_byte(bytes[3]); // MSB
    //send_crc();

    uint16_t type;
    uint32_t value;
    uint8_t checksum;

    bytes[0] = _port->read();
    // Check if its the sport frame header, if so then read further values.
    if(bytes[0] == 0x10) {
        bytes = (uint8_t*)&type;
        bytes[0] = _port->read();
        bytes[1] = _port->read();

        bytes = (uint8_t*)&value;
        bytes[0] = _port->read();
        bytes[1] = _port->read();
        bytes[2] = _port->read();
        bytes[3] = _port->read();

        bytes = (uint8_t*)&checksum;
        bytes[0] = _port->read();
        bytes[1] = _port->read();
        gcs().send_text(MAV_SEVERITY_INFO, "FR: %d", type);
    } else {

        gcs().send_text(MAV_SEVERITY_INFO, "FR: Unknown: %d", bytes[0]);
    }
    //if(type == )
}


/*
 * tick - main call to send data to the receiver (called by scheduler at 1kHz)
 */
void AP_Frsky_Sensor::tick(void)
{
    // Only update once every 100ms
    uint32_t now = AP_HAL::millis();
    if (now < (last_called_ms + 500)) {
        return;
    }
    last_called_ms = now;

    //gcs().send_text(MAV_SEVERITY_INFO, "FR: Cell1: %d", mlvss.cell1);
    mlvss.cell1++;

    get(1);
}

void AP_Frsky_Sensor::calc_crc(uint8_t byte)
{
    _crc += byte; //0-1FF
    _crc += _crc >> 8; //0-100
    _crc &= 0xFF;
}

void AP_Frsky_Sensor::send_crc(void)
{
    send_byte(0xFF - _crc);
    _crc = 0;
}



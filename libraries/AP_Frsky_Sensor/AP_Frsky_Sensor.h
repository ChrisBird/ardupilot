/*
   Please contribute your ideas! See http://dev.ardupilot.org for details

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
//#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Frsky_Telem/AP_Frsky_Telem.h>

class AP_Frsky_Sensor {

public:
    AP_Frsky_Sensor();

    static AP_Frsky_Sensor *get_singleton(void) {
        return _singleton;
    }

    enum Frsky_Sensor_Types {
        VLSS=0x00,
    };

    void init();
    void update();

    uint8_t num_sensors() const {return num_instances;}

    uint16_t mlvss_Voltage_Sum() { return mlvss.cell1 + mlvss.cell2 + mlvss.cell3 + mlvss.cell4 + mlvss.cell5 + mlvss.cell6; }
protected:

private:
    static AP_Frsky_Sensor *_singleton;

    uint8_t num_instances;

    struct frsky_sensor {
        uint8_t     sensor_id;
        uint16_t    last_type;
        uint32_t    last_value;
    };

    struct PACKED raw_frsky_sensor_data_rx {
        uint8_t     frame_header;
        uint16_t    type;
        uint32_t    value;
        uint8_t     checksum;
    };

    struct PACKED raw_frsky_sensor_data_tx {
        uint8_t     start_stop;
        uint8_t     sensor_id;
    };

    struct frsky_mlvss {
        uint16_t cell1;
        uint16_t cell2;
        uint16_t cell3;
        uint16_t cell4;
        uint16_t cell5;
        uint16_t cell6;
    } mlvss;

    AP_HAL::UARTDriver *_port;
    uint16_t _crc;

    uint32_t last_called_ms;

    void get(uint8_t id);
    void send_byte(uint8_t byte);
    void tick(void);
    void calc_crc(uint8_t byte);
    void send_crc(void);
};

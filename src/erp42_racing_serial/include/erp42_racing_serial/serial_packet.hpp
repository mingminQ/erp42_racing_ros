/**
 * -------------------------------------------------------------------------------------------------
 * 
 * Copyright 2025 Minkyu Kil
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 * @file    serial_packet.hpp
 * @brief   ERP42 Racing serial packet byte name wrapper and factors
 * @author  Minkyu Kil
 * @date    2025-06-17
 * @version 1.0
 *
 * -------------------------------------------------------------------------------------------------
 */

#ifndef ERP42_RACING_SERIAL__SERIAL_PACKET_HPP_
#define ERP42_RACING_SERIAL__SERIAL_PACKET_HPP_

namespace erp42_racing::serial
{
    // ERP42 serial tx packet index mapping
    namespace TX
    {
        enum ByteName
        {
            STX_S          = 0,
            STX_T          = 1,
            STX_X          = 2,
            CONTROL_MODE   = 3,
            EMERGENCY_STOP = 4,
            GEAR           = 5,
            SPEED_RAW_0    = 6,
            SPEED_RAW_1    = 7,
            STEERING_100_0 = 8,
            STEERING_100_1 = 9,
            BRAKE          = 10,
            HEARTBEAT      = 11,
            ETX_0          = 12,
            ETX_1          = 13,
            PACKET_SIZE    = 14

        }; // enum ByteName

    } // namespace TX

    // ERP42 serial rx packet index mapping
    namespace RX
    {
        enum ByteName
        {
            STX_S          = 0,
            STX_T          = 1,
            STX_X          = 2,
            CONTROL_MODE   = 3,
            EMERGENCY_STOP = 4,
            GEAR           = 5,
            SPEED_RAW_0    = 6,
            SPEED_RAW_1    = 7,
            STEERING_100_0 = 8,
            STEERING_100_1 = 9,
            BRAKE          = 10,
            ENCODER_0      = 11,
            ENCODER_1      = 12,
            ENCODER_2      = 13,
            ENCODER_3      = 14,
            BATTERY_0      = 15,
            BATTERY_1      = 16,
            HEARTBEAT      = 17,
            ETX_0          = 18,
            ETX_1          = 19,
            PACKET_SIZE    = 20

        }; // enum ByteName

    } // namespace RX

    // Speed(m/s) -> Raw byte command
    static constexpr double MPS2BYTE {219.4};

    // Steering(rad) -> Raw byte command
    static constexpr double RAD2BYTE {-5729.57795131};

    // Raw byte command -> Speed (m/s)
    static constexpr double BYTE2MPS {0.03036872898};

    // Raw byte command -> Steering (rad)
    static constexpr double BYTE2RAD {0.00017453292};

    // Raw battery -> Battery Voltage (V)
    static constexpr double BYTE2BAT {0.1};

} // namespace erp42_racing::serial

#endif // ERP42_RACING_SERIAL__SERIAL_PACKET_HPP_
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
 * @file    vehicle_parameters.hpp
 * @brief   ERP42 Racing platform vehicle parameters
 * @author  Minkyu Kil
 * @date    2025-06-17
 * @version 1.0
 *
 * -------------------------------------------------------------------------------------------------
 */

#ifndef ERP42_RACING_DESCRIPTION__VEHICLE_PARAMETERS_HPP_
#define ERP42_RACING_DESCRIPTION__VEHICLE_PARAMETERS_HPP_

namespace erp42_racing_description
{
    // ERP42 RACING kinematic parameters
    static constexpr double BASE_X_SIZE {2.060};
    static constexpr double BASE_Y_SIZE {1.160};
    static constexpr double BASE_Z_SIZE {0.822};

    static constexpr double WHEELBASE_LENGTH  {1.212};
    static constexpr double FRONT_TRACK_WIDTH {0.938};
    static constexpr double REAR_TRACK_WIDTH  {0.970};

    static constexpr double FRONT_WHEEL_RADIUS {0.290};
    static constexpr double FRONT_WHEEL_WIDTH  {0.200};
    static constexpr double REAR_WHEEL_RADIUS  {0.280};
    static constexpr double REAR_WHEEL_WIDTH   {0.170};

    // Encoder parameters
    static constexpr double ENCODER_PPR {48.0};
    static constexpr double ENCODER_CPR {192.0};

} // namespace erp42_racing_description

#endif // ERP42_RACING_DESCRIPTION__VEHICLE_PARAMETERS_HPP_
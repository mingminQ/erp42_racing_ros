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
 * @file    erp42_racing_parameters.hpp
 * @brief   ERP42 Racing platform vehicle parameters
 * @author  Minkyu Kil
 * @date    2025-07-01
 * @version 1.0
 *
 * -------------------------------------------------------------------------------------------------
 */

#ifndef ERP42_RACING_UTIL__ERP42_RACING_PARAMETERS_HPP_
#define ERP42_RACING_UTIL__ERP42_RACING_PARAMETERS_HPP_

namespace erp42_racing_util
{
    static constexpr double CHASSIS_LENGTH {2.060};
    static constexpr double CHASSIS_WIDTH  {1.160};
    static constexpr double CHASSIS_HEIGHT {0.822};

    static constexpr double WHEELBASE   {1.212};
    static constexpr double FRONT_TREAD {0.938};
    static constexpr double REAR_TREAD  {0.970};

    static constexpr double FRONT_WHEEL_RADIUS {0.290};
    static constexpr double FRONT_WHEEL_WIDTH  {0.200};
    static constexpr double REAR_WHEEL_RADIUS  {0.280};
    static constexpr double REAR_WHEEL_WIDTH   {0.170};

    static constexpr double MAX_STEERING {0.34906585039};
    static constexpr double MAX_SPEED    {5.5};

    static constexpr double ENCODER_PPR {48.0};
    static constexpr double ENCODER_CPR {192.0};

} // namespace erp42_racing_util

#endif // ERP42_RACING_UTIL__ERP42_RACING_PARAMETERS_HPP_
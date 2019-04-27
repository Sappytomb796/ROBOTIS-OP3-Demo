/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
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
*******************************************************************************/

// Author: Mel Flygare

#include "op3_ball_detector/ball_detector_config.h"

namespace robotis_op
{

int BallColorConfig::sampleLightVal()
{
    return rand() % x_max + x_min;
}

int BallColorConfig::getMedianBVal(int x_val)
{
    return light_slope * x_val + light_constant;
}

// int modifyHSV(int R)
// {

// }

}
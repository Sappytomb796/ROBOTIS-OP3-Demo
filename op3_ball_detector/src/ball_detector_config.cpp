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

BallColorConfig::BallColorConfig()
    : x_min(X_MIN_DEFAULT),
      x_max(X_MAX_DEFAULT),
      light_slope(LIGHT_SLOPE_DEFAULT),
      light_constant(LIGHT_CONSTANT_DEFAULT)
  {
    std::random_device rd;
    gen.seed(rd);
  }

int BallColorConfig::sampleLightVal()
{
    return light_distribution(gen);
}

int BallColorConfig::getMedianRVal(int x_val)
{
    return light_slope * x_val + light_constant;
}

void BallColorConfig::updateDistribution(std::vector<double> light_range, std::vector<double> range_weights)
{
    light_distribution.param({light_range.begin(), light_range.end(), range_weights.begin()});
}

}
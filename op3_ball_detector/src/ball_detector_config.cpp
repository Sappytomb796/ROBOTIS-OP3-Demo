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
#include <iostream>

namespace robotis_op
{

BallColorConfig::BallColorConfig()
    : x_min(X_MIN_DEFAULT),
      x_max(X_MAX_DEFAULT),
      light_slope(LIGHT_SLOPE_DEFAULT),
      light_constant(LIGHT_CONSTANT_DEFAULT)
{
    std::random_device rd;
    gen.seed(rd());
    srand(time(NULL));
}

int BallColorConfig::sampleLightVal()
{
    // Small chance to sample value outside of the distribution
    // to avoid getting stuck in maximum
    if((rand() % 100) <= (RANDOM_SAMPLE_CHANCE * 100))
    {
        std::cout << "BOO!" << std::endl;
        return rand() % (x_max + x_min);
    }
    return light_distribution(gen);
}

int BallColorConfig::getMedianRVal(int x_val)
{
    return light_slope * x_val + light_constant;
}

void BallColorConfig::updateDistribution(std::vector<double> light_range, std::vector<double> light_weights)
{
    light_distribution.param(std::piecewise_constant_distribution<>::param_type(light_range.begin(), light_range.end(), light_weights.begin()));
}

void BallColorConfig::adjustWeightsWithLightVal(int light_val, int adjust_val, std::vector<double> &light_weights)
{
    if(light_weights[int((light_val - x_min) / (range / NUM_INTERVALS))] + adjust_val > 0)
        light_weights[int((light_val - x_min) / (range / NUM_INTERVALS))] += adjust_val;
    else
        light_weights[int((light_val - x_min) / (range / NUM_INTERVALS))] = 1;
}

}
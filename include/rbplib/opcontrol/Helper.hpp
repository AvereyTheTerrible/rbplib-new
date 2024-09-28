//
// Created by aeku on 9/1/24.
//

#pragma once

#include <cmath>

namespace rbplib
{
    template <typename T> int sgn(T val) 
    {
        return (T(0) < val) - (val < T(0));
    }


    double curve(double input, double deadband = 3, double curveGain = 1.025, double minOutput = 4)
    {
        // return 0 if input is within deadzone
        if (fabs(input) <= deadband) return 0;
        // g is the output of g(x) as defined in the Desmos graph
        const float g = fabs(input) - deadband;
        // g127 is the output of g(127) as defined in the Desmos graph
        const float g127 = 127 - deadband;
        // i is the output of i(x) as defined in the Desmos graph
        const float i = pow(curveGain, g - 127) * g * sgn(input);
        // i127 is the output of i(127) as defined in the Desmos graph
        const float i127 = pow(curveGain, g127 - 127) * g127;
        return (127.0 - minOutput) / (127) * i * 127 / i127 + minOutput * sgn(input);
    }
} // namespace rbplib
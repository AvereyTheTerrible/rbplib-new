//
// Created by aeku on 9/13/24.
//

#include "ThreeWheelOdometry.hpp"

namespace rbplib
{
    ThreeWheelOdometry::ThreeWheelOdometry(const std::shared_ptr<pros::Rotation> &ihorizontalEncoder,
                           const std::shared_ptr<pros::MotorGroup> &ileft,
                           const std::shared_ptr<pros::MotorGroup> &iright,
                           const double iencoderWheelRadius,
                           const double itrackWidth)
    : horizontalEncoder(ihorizontalEncoder),
      left(ileft),
      right(iright),
      encoderWheelRadius(iencoderWheelRadius),
      trackWidth(itrackWidth)
    {

    }

    void ThreeWheelOdometry::step()
    {
		pose(2) = 
    }
} // rbplib
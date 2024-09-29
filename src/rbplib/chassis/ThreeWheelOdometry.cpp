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
		pose(2) = (currentLeftAngularPosition - lastLeftAngularPosition + currentRightAngularPosition - lastRightAngularPosition)/trackWidth;
    pose(1) = 2 * ((currentRightAngularPosition - lastRightAngularPosition)/pose(2)+ trackWidth) * sin(pose(2)/2);
    pose(0) = 2* ((currentHorizontalAngularPosition - lastHorizontalAngularPosition)/pose(2) + encoderWheelRadius) * sin(pose(2)/2);
    }
} // rbplib
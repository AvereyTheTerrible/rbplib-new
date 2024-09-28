//
// Created by aeku on 9/13/24.
//

#pragma once

#include "api.h"

namespace rbplib
{
class ThreeWheelOdometry
{
    public:
        ThreeWheelOdometry(const std::shared_ptr<pros::Rotation> &ihorizontalEncoder,
                           const std::shared_ptr<pros::MotorGroup> &ileft,
                           const std::shared_ptr<pros::MotorGroup> &iright,
                           const double iencoderWheelRadius,
                           const double itrackWidth);

        void run();

    private:
        void step();

        std::shared_ptr<pros::Rotation> horizontalEncoder;
        std::shared_ptr<pros::MotorGroup> right, left;
        double encoderWheelRadius,
               trackWidth;

        double lastLeftAngularPosition,
               lastRightAngularPosition,
               lastHorizontalAngularPosition,

               currentLeftAngularPosition,
               currentRightAngularPosition,
               currentHorizontalAngularPosition;

        Eigen::Vector3d pose;
};

} // rbplib
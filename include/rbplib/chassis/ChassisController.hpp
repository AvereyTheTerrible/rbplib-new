//
// Created by aeku on 9/12/24.
//

#pragma once

#include "rbplib/chassis/DifferentialDriveKinematics.hpp"
#include "api.h"
#include <memory>

namespace rbplib {
class ChassisController {
public:
    ChassisController(const std::shared_ptr<pros::MotorGroup> &ileft,
                      const std::shared_ptr<pros::MotorGroup> &iright,
                      const double trackWidth,
                      const double wheelRadius,
                      const double gearRatio=1.0);

    void drive(const Eigen::Vector2d &velocity);

private:
    std::shared_ptr<pros::MotorGroup> left, right;
    std::shared_ptr<DifferentialDriveKinematics> kinematics;

    const double trackWidth, wheelRadius;
};

} // rbplib

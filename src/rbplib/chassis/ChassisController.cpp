//
// Created by aeku on 9/12/24.
//

#include "ChassisController.hpp"

namespace rbplib
{
      ChassisController::ChassisController(const std::shared_ptr<pros::MotorGroup> &ileft,
                                           const std::shared_ptr<pros::MotorGroup> &iright,
                                           const double itrackWidth,
                                           const double iwheelRadius,
                                           const double gearRatio)
      : left(ileft),
        right(iright),
        trackWidth(itrackWidth),
        wheelRadius(iwheelRadius)
      {
          double motorRPM = 0.0;
          switch (left->get_gearing())
          {
              case pros::MotorGears::red:
                  motorRPM = 100.0;
                  break;
                case pros::MotorGears::green:
                  motorRPM = 200.0;
                  break;
              case pros::MotorGears::blue:
                  motorRPM = 600.0;
                  break;
              case pros::MotorGears::invalid:
                  motorRPM = 0.0;
                  break;
          }

          kinematics = std::make_shared<DifferentialDriveKinematics>(itrackWidth, wheelRadius, motorRPM, gearRatio);
      }

      void ChassisController::drive(const Eigen::Vector2d &velocity)
      {
          auto wheelVelocities = kinematics->inverseKinematics(velocity);

          right->move_velocity(wheelVelocities(0));
          left->move_velocity(wheelVelocities(1));
      }
} // rbplib
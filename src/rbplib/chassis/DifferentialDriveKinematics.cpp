//
// Created by aeku on 9/1/24.
//

#include "rbplib/chassis/DifferentialDriveKinematics.hpp"

namespace rbplib
{
    DifferentialDriveKinematics::DifferentialDriveKinematics(const double itrackWidth,
                                                         	 const double iwheelRadius,
                                                         	 const double imotorVelocity,
                                                         	 const double igearRatio)
	: trackWidth(itrackWidth),
      wheelRadius(iwheelRadius),
      maxLinearSpeed(imotorVelocity * igearRatio * iwheelRadius),
      maxAngularSpeed(2 * maxLinearSpeed / trackWidth)
    {
    }

	Eigen::Vector2d DifferentialDriveKinematics::forwardKinematics(const Eigen::Vector2d &wheelVelocities)
    {
        double linear = (wheelVelocities(0) + wheelVelocities(1) * wheelRadius / 2),
               angular = (wheelVelocities(1) - wheelVelocities(0) * wheelRadius / trackWidth);

        return Eigen::Vector2d(linear, angular);
    }

	Eigen::Vector2d DifferentialDriveKinematics::inverseKinematics(const Eigen::Vector2d &velocity)
    {
       	auto scaledLinVelocity = velocity(0) * maxLinearSpeed,
             scaledAngVelocity = velocity(1) * maxAngularSpeed;

        double left = scaledLinVelocity / wheelRadius - (trackWidth * scaledAngVelocity) / (2 * wheelRadius),
               right = scaledLinVelocity / wheelRadius + (trackWidth * scaledAngVelocity) / (2 * wheelRadius);

        return Eigen::Vector2d(left * 9.55, right * 9.55);
    }
} // rbplib
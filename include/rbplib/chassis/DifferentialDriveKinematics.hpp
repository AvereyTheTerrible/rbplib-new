//
// Created by aeku on 9/1/24.
//

#pragma once

#include "rbplib/chassis/ChassisKinematics.hpp"

namespace rbplib
{
class DifferentialDriveKinematics : public ChassisKinematics
{

public:
	DifferentialDriveKinematics(const double itrackWidth,
                              	const double iwheelRadius,
                              	const double imotorVelocity,
                              	const double igearRatio=1.0);

    /**
	 * Calculates the forward kinematics of a differential drive.
     *
     * @param wheelVelocities the left and right wheel velocities packaged as a vector with each dimension scaled to [-1.0, 1.0]
     */
	Eigen::Vector2d forwardKinematics(const Eigen::Vector2d &wheelVelocities) override;

    /**
	 * Calculates inverse kinematics of a differential drive.
     *
     * @param velocity linear and angular velocities packaged as a vector with each dimension scaled to [-1.0, 1.0]
	 */
	Eigen::Vector2d inverseKinematics(const Eigen::Vector2d &velocity) override;

private:
	const double trackWidth, wheelRadius, maxLinearSpeed, maxAngularSpeed;
};
} // rbplib

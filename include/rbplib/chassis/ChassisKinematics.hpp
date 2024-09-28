//
// Created by aeku on 9/1/24.
//

#pragma once

#include "Eigen/Dense"

// virtual class with forward and inverse kinematics for a chassis in a body-attached frame.
class ChassisKinematics
{
public:
	ChassisKinematics() = default;

	virtual Eigen::Vector2d forwardKinematics(const Eigen::Vector2d &wheelVelocities) = 0;

    virtual Eigen::Vector2d inverseKinematics(const Eigen::Vector2d &velocity) = 0;
};

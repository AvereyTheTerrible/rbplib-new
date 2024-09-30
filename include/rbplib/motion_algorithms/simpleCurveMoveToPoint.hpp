//
// Created by aeku on 9/13/24.
//

#pragma once

#include "api.h"
#include "rbplib\chassis\DifferentialDriveKinematics.hpp"

namespace rbplib
{
class simpleCurveMoveToPoint
{
    public:
        void simpleCurveMoveToPoint(Vector2d target,
            Vector3d init_pose)
            {
                double interval = 0.01;
                double dist = sqrt((init_pose(1)-target(1))**2 + (init_pose(0)-target(0))**2);
                double angle = pose(2) - atan2((init_pose(1) - target(1))/(init_pose(0)-target(0)))
                Eigen::Vector2d vec = {dist/interval, angle/interval};
                inverseKinematics(dist/interval, angle/interavl);
            }

        void run();

    private:
        void step();
        Eigen::Vector3d pose;
};

} // rbplib
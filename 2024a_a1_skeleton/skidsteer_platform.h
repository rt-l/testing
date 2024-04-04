#ifndef SKIDSTEER_PLATFORM_H
#define SKIDSTEER_PLATFORM_H

#include "controllerinterface.h"
#include <cmath>

class SkidSteerPlatform : public ControllerInterface
{
    private:
        const double MAX_LINEAR_VELOCITY = 1.0;
        const double MAX_ANGULAR_VELOCITY = 1.0;

        const double WEIGHT = 200.0;

    public:
        SkidSteerPlatform(){}

        bool reachGoal(void) override
        {

        }

        bool setGoal(pfms::geometry_msgs::Point goal) override
        {

        }

        bool checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                    pfms::geometry_msgs::Point goal,
                                    double &distance,
                                    double &time,
                                    pfms::nav_msgs::Odometry &estimatedGoalPose) override
        {

        }

        pfms::PlatformType getPlatformType(void) override
        {
            return pfms::PlatformType::SKIDSTEER;
        }

        double distanceToGoal(void) override
        {

        }

        double timeToGoal(void) override
        {

        }

        bool setTolerance(double tolerance) override
        {

        }
            
        double distanceTravelled(void) override
        {

        }

        double timeInMotion(void) override
        {

        }

        pfms::nav_msgs::Odometry getOdometry(void) override
        {

        }
};

#endif // SKIDSTEER_PLATFORM_H
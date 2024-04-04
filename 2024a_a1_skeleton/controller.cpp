#include "controller.h"
#include <iostream>

Controller::Controller(pfms::PlatformType platformType) : platformType_(platformType)
{

}

Controller::~Controller()
{

}

bool Controller::reachGoal()
{
    
}

bool Controller::setGoal(pfms::geometry_msgs::Point goal)
{
    goal_ = goal;
    return true;
}

bool Controller::checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                        pfms::geometry_msgs::Point goal,
                                        double& distance,
                                        double& time,
                                        pfms::nav_msgs::Odometry& estimatedGoalPose)
{
    return true;
}

pfms::PlatformType Controller::getPlatformType()
{
    return platformType_;
}

double Controller::timeToGoal()
{
    return 0.0;
}

bool Controller::setTolerance(double tolerance)
{
    return true;
}

double Controller::distanceTravelled()
{
    return 0.0;
}

double Controller::timeInMotion()
{
    return 0.0;
}

pfms::nav_msgs::Odometry Controller::getOdometry()
{
    pfms::nav_msgs::Odometry odometry;
    return odometry;
}
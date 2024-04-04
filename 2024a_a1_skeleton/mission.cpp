#include "mission.h"

Mission::Mission(std::vector<ControllerInterface*> controllers) : controllers_(controllers)
{

}

// Mission::Mission(std::vector<ControllerInterface*> controllers) : controllers_(controllers), objective_(mission::Objective::DISTANCE)
// {

// }
void Mission::setGoals(std::vector<pfms::geometry_msgs::Point> goals)
{
    goals_.clear();

    goals_ = goals;
}

bool Mission::runMission()
{
    distance_travelled_.clear();
    time_moving_.clear();
    platform_goal_association_.clear();

    for(auto controller : controllers_)
    {
        double distance = controller->distanceToGoal();
        double time = controller->timeToGoal();
        pfms::nav_msgs::Odometry estimated_goal_pose = controller->getOdometry();

        distance_travelled_.push_back(distance);
        time_moving_.push_back(time);
        platform_goal_association_.push_back(0);
    }

    return true;
}

void Mission::setMissionObjective(mission::Objective objective)
{
    objective_ = objective;
}

std::vector<double> Mission::getDistanceTravelled()
{
    return distance_travelled_;
}

std::vector<double> Mission::getTimeMoving()
{
    return time_moving_;
}

std::vector<unsigned int> Mission::getPlatformGoalAssociation()
{
    return platform_goal_association_;
}

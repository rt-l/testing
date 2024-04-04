#include "ackerman.h"
#include "pfmsconnector.h"
#include <chrono>
#include <cmath>
#include <pfms_types.h>


Ackerman::Ackerman()
{
    type_ = pfms::PlatformType::ACKERMAN;
    steering_ratio_ = 17.3;
    lock_to_lock_revs_ = 3.2;
    max_steer_angle_ = (M_PI * lock_to_lock_revs_/steering_ratio_);
    wheelbase_ = 2.65;
    trackwidth_ = 1.638;
    max_break_torque_ = 8000.0;
    steadyStateV_ = 2.91;
    throttle_ = 0.1;
    prevD_ = 0.0;
    PfmsConnector connector_;
    

}

bool Ackerman::reachGoal(void)
{
    reach_goal = false;
    unsigned long i = 0;
    brake_ = 0;
    throttle_ = 0.1;
    double steering_ = delta_ * steering_ratio_;

    while (!reach_goal)
    {
        
        bool ok = connector_.read(odo, type);

        current_odometry = getOdometry();

        double dx = goal_.x - current_odometry.position.x;
        double dy = goal_.y - current_odometry.position.y;
        double current_displacement = sqrt(pow(dx, 2) + pow(dy, 2));

        if(current_displacement <= tolerance_)
        {
            brake_ = max_break_torque_;
            throttle_ = 0.0;
        }

        pfms::commands::Ackerman ackerman_cmd;
        ackerman_cmd.seq = i;
        ackerman_cmd.throttle = throttle_;
        ackerman_cmd.brake = brake_;
        ackerman_cmd.steering = steering_;

        connector_.send(ackerman_cmd);
        ++i;

        if(i > 20 && current_odometry.linear.x <= 0.02 && current_odometry<= 0.02)
        {
            reach_goal = true;
            break;
        }
    }

    if(i > 700)
    {
        break;
    }
}

return reach_goal;

bool Ackerman::checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                        pfms::geometry_msgs::Point goal,
                                        double& distance,
                                        double& time,
                                        pfms::nav_msgs::Odometry& estimatedGoalPose)
{
    double dx = goal.x - origin.position.x;
    double dy = goal.y - origin.position.y; 
    double str8dist = std::sqrt(dx * dx + dy * dy);
    double heading = std::atan2(dx,dy);
    distance = str8dist * heading;
    time = distance / 2.91;

    estimatedGoalPose.position.x = goal.x;
    estimatedGoalPose.position.y = goal.y;
    estimatedGoalPose.yaw = origin.yaw + heading;

    double angleDiff = std::abs(heading - origin.yaw);

    if(angleDiff <= max_steer_angle_)
    {
        return true;
    }

    else false;

}
#ifndef ACKERMAN_H
#define ACKERMAN_H

#include "controller.h"

class Ackerman: public Controller
{
public:
  //Default constructor should set all attributes to a default value
  Ackerman();

  bool Ackerman::reachGoal(void);

  bool checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                        pfms::geometry_msgs::Point goal,
                                        double& distance,
                                        double& time,
                                        pfms::nav_msgs::Odometry& estimatedGoalPose);

  double calculateArcLength(pfms::geometry_msgs::Point& origin, pfms::geometry_msgs::Point& goal);

private:
  pfms::PlatformType type_;
  double steering_ratio_;
  double lock_to_lock_revs_;
  double max_steer_angle_;
  double wheelbase_;
  double trackwidth_;
  double max_break_torque_;
  double steadyStateV_;
  double throttle_;
  double prevD_;
  double brake_;
  bool reach_goal;
  PfmsConnector* connector_;
};

#endif // ACKERMAN_H

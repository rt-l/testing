#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "controllerinterface.h"
#include "pfmsconnector.h"

class Controller: public ControllerInterface
{
public:
  //Default constructors should set all attributes to a default value
  Controller();
  ~Controller();
  Controller(pfms::PlatformType platformType);

  bool reachGoal(void) override;
  bool setGoal(pfms::geometry_msgs::Point goal) override;
  bool checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                pfms::geometry_msgs::Point goal,
                                double& distance,
                                double& time,
                                pfms::nav_msgs::Odometry& estimatedGoalPose) override;
  pfms::PlatformType getPlatformType(void) override;
  double distanceToGoal(void) override;
  double timeToGoal(void) override;
  bool setTolerance(double tolerance) override;
  double distanceTravelled(void) override;
  double timeInMotion(void) override;
  pfms::nav_msgs::Odometry getOdometry(void) override;

protected:
  
  pfms::PlatformType platformType_;
  bool reach_goal_;

  //See controllerinterface.h for more information
};

#endif // CONTROLLER_H

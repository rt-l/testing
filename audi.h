#ifndef AUDI_H
#define AUDI_H

#include "pfms_types.h"

/*!
 *  \brief     Audi Class
 *  \details
 *  This class implements computing steering to drive towards a goal from current odometry
 *  It also provides capability to compute distance and time from origin to goal
 *  \author    Alen Alempijevic
 *  \version   1.00-1
 *  \date      2022-04-15
 *  \pre       none
 *  \bug       none reported as of 2022-04-15
 *  \warning   students MUST NOT change this class (the header file)
 */


class Audi
{
public:
  //Default constructor should set all attributes to default audi settings
  Audi();
  ~Audi();

  /**
  Checks whether the platform can travel between origin and destination via constant steering angle
  @param[in] origin The origin pose, specified as odometry for the platform
  @param[in] goal The destination point for the platform
  @param[in|out] distance The distance [m] the platform will need to travel between origin and destination. If destination unreachable distance = -1
  @param[in|out] time The time [s] the platform will need to travel between origin and destination, If destination unreachable time = -1
  @param[in|out] estimatedGoalPose The estimated goal pose when reaching goal
  @return bool indicating the platform can reach the destination from origin supplied
  */
  bool checkOriginToDestination(pfms::nav_msgs::Odometry origin, pfms::geometry_msgs::Point goal,
                                 double& distance, double& time, pfms::nav_msgs::Odometry& estimatedGoalPose);

  /**
  Checks whether the platform can travel between origin and destination via constant steering angle
  @param[in] origin The origin pose, specified as odometry for the platform
  @param[in] goal The destination point for the platform
  @param[in|out] steering The steering needed to reach the goal (constant steering)
  @param[in|out] dist The distance along the arc travelled to the goal
  @return bool indicating the platform can reach the goal from odo supplied
  */
  bool computeSteering(pfms::nav_msgs::Odometry origin, pfms::geometry_msgs::Point goal,
                      double& steering,double& dist);

private:
  pfms::PlatformType type_;
  const double steering_ratio_;
  const double lock_to_lock_revs_;
  const double max_steer_angle_;
  const double wheelbase_;
  const double max_break_torque_;
  const double steadyStateV_;
  const double deltaD_;
  double prevD_;

};

#endif // Audi_H

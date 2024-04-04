#ifndef A1_TEST_HELPER_H
#define A1_TEST_HELPER_H

#include "pfms_types.h"
#include <cmath>

using namespace pfms::nav_msgs;

/////////////////////////////////////////////////////////////////////////////////////
/// \brief populateOdo
/// \param x - position x
/// \param y - position y
/// \param yaw - yaw in radians
/// \return assembled odo message (with zero for velocity)
Odometry populateOdo(double x, double y, double yaw){
    Odometry odo;
    odo.position.x=x;
    odo.position.y=y;
    odo.yaw=yaw;
    odo.linear.x=0;
    odo.linear.y=0;
    return odo;
}

#endif // A1_TEST_HELPER_H

#ifndef _SUBSUMPTION_HPP
#define _SUBSUMPTION_HPP

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

///////////////////////////////////////////////////////////////
// Constants
#define TIME_INT 0.1
#define DIST_OBS 0.5
#define VEL_OBS 0.5
#define DIST_OFF_ALT 0.5
#define VEL_ALT 0.5

#define DIST_RATE_TURN 1.1
#define VEL_TURN 1.0

#define DIST_RATE_ADJL 0.9
#define DIST_RATE_ADJR 1.1

#define DIST_MAX 10.0
#define DIST_OFF 1.0
#define VEL_MIDDLE 0.5

#define VEL_STRAIGHT 0.8
#define VEL_FIND 1.0

///////////////////////////////////////////////////////////////
// INPUT
// structure of INPUT which holds all necessary sensor data
struct INPUT
{
  double h[5];
  double u[5];
  double d[5];
};

///////////////////////////////////////////////////////////////
// getNextCom
// it generates a command from the given input
// based on the subsumption concept.
//
geometry_msgs::Twist getNextCom(INPUT);

#endif // _SUBSUMPTION_HPP


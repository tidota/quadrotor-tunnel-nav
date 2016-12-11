#include "subsumption.hpp"

///////////////////////////////////////////////////////////////
// getNextCom
// it generates a command from the given input
// based on the subsumption concept.
//
geometry_msgs::Twist getNextCom(INPUT input)
{
  geometry_msgs::Twist com;
  com.linear.x = 0;
  com.linear.y = 0;
  com.linear.z = 0;
  com.angular.x = 0;
  com.angular.y = 0;
  com.angular.z = 0;

  bool f_fired = false;
  bool f_done = false;

// OBSTACLE AVOIDANCE
  // input check
  f_fired = (
    input.h[0] < DIST_OBS || input.h[1] < DIST_OBS || input.h[2] < DIST_OBS || input.h[3] < DIST_OBS || input.h[4] < DIST_OBS || 
    input.d[1] < DIST_OBS || input.d[2] < DIST_OBS || input.d[3] < DIST_OBS || input.d[4] < DIST_OBS || 
    input.u[1] < DIST_OBS || input.u[2] < DIST_OBS || input.u[3] < DIST_OBS); 
  if(f_fired == true && f_done == false)
  {
    ROS_INFO("OBSTACLE AVOIDANCE");
    // calculate the output
    if(input.h[0] < DIST_OBS)
    {
      com.linear.y += VEL_OBS;
    }
    if(input.h[1] < DIST_OBS)
    {
      com.linear.x -= VEL_OBS/sqrt(2);
      com.linear.y += VEL_OBS/sqrt(2);
    }
    if(input.h[2] < DIST_OBS)
    {
      com.linear.x -= VEL_OBS;
    }
    if(input.h[3] < DIST_OBS)
    {
      com.linear.x -= VEL_OBS/sqrt(2);
      com.linear.y -= VEL_OBS/sqrt(2);
    }
    if(input.h[4] < DIST_OBS)
    {
      com.linear.y -= VEL_OBS;
    }
    if(input.d[1] < DIST_OBS)
    {
      com.linear.x -= VEL_OBS/sqrt(2);
      com.linear.z += VEL_OBS/sqrt(2);
    }
    if(input.d[2] < DIST_OBS)
    {
      com.linear.z += VEL_OBS;
    }
    if(input.d[3] < DIST_OBS)
    {
      com.linear.x += VEL_OBS/sqrt(2);
      com.linear.z += VEL_OBS/sqrt(2);
    }
    if(input.d[4] < DIST_OBS)
    {
      com.linear.x += VEL_OBS;
    }
    if(input.u[1] < DIST_OBS)
    {
      com.linear.x += VEL_OBS/sqrt(2);
      com.linear.z -= VEL_OBS/sqrt(2);
    }
    if(input.u[2] < DIST_OBS)
    {
      com.linear.z -= VEL_OBS;
    }
    if(input.u[3] < DIST_OBS)
    {
      com.linear.x -= VEL_OBS/sqrt(2);
      com.linear.z -= VEL_OBS/sqrt(2);
    }

    f_done = true;
  }

// KEEP THE ALTITUDE
  // input check
  f_fired = input.d[2] - input.u[2] > DIST_OFF_ALT || input.u[2] - input.d[2] > DIST_OFF_ALT; 
  if(f_fired == true && f_done == false)
  {
    ROS_INFO("KEEP THE ALTITUDE");
    // calculate the output
    com.linear.z = VEL_ALT * (input.d[2] < input.u[2])? 1: -1;
    if(input.h[2] > input.h[0])
      com.linear.x = VEL_STRAIGHT;
    f_done = true;
  }

// TURN RIGHT
  // if the wall in front is closer than the right one
  // and the right-front space appears open,
  // turn to the right

  // input check
  f_fired = 
    input.h[0] > input.h[2] &&
    input.h[1] > input.h[0] * sqrt(2) * DIST_RATE_TURN;
  if(f_fired == true && f_done == false)
  {
    ROS_INFO("TURN RIGHT");
    // calculate the output
    com.angular.z = -VEL_TURN;
    f_done = true;
  }

// TURN LEFT
  // if the wall in front is closer than the right one
  // and the right-front space appears blocked,
  // turn to the left

  // input check
  f_fired = 
    input.h[0] > input.h[2] &&
    input.h[1] <= input.h[0] * sqrt(2) * DIST_RATE_TURN;
  if(f_fired == true && f_done == false)
  {
    ROS_INFO("TURN LEFT");
    // calculate the output
    com.angular.z = VEL_TURN;
    f_done = true;
  }

// ADJUST TO THE LEFT
  // input check
  f_fired = 
    input.h[1] < input.h[0] * sqrt(2) * DIST_RATE_ADJL;
  if(f_fired == true && f_done == false)
  {
    ROS_INFO("ADJUST TO THE LEFT");
    // calculate the output
    com.angular.z = VEL_TURN;
    com.linear.x = VEL_STRAIGHT;
    f_done = true;
  }

// ADJUST TO THE RIGHT
  // input check
  f_fired = 
    input.h[1] > input.h[0] * sqrt(2) * DIST_RATE_ADJR;
  if(f_fired == true && f_done == false)
  {
    ROS_INFO("ADJUST TO THE RIGHT");
    // calculate the output
    com.angular.z = -VEL_TURN;
    com.linear.x = VEL_STRAIGHT;
    f_done = true;
  }

// STAY ON THE MIDDLE LINE
  // input check
  f_fired = 
    input.h[4] <= input.h[2] &&
    (input.h[0] < input.h[4] - DIST_OFF ||
     input.h[0] > input.h[4] + DIST_OFF); 
  if(f_fired == true && f_done == false)
  {
    ROS_INFO("STAY ON THE MIDDLE LINE");
    // calculate the output
    com.linear.y = (input.h[0] < input.h[4])? VEL_MIDDLE: -VEL_MIDDLE;
    f_done = true;
  }
    
// GO STRAIGHT
  // input check
  f_fired = input.h[0] < DIST_MAX;
  if(f_fired == true && f_done == false)
  {
    ROS_INFO("GO STRAIGHT");
    // calculate the output
    com.linear.x = VEL_STRAIGHT;
    f_done = true;
  }

// FIND A WALL
  // input check
  f_fired = true;
  if(f_fired == true && f_done == false)
  {
    ROS_INFO("FIND A WALL");
    // calculate the output
    com.linear.y = -VEL_FIND;
    f_done = true;
  }

  return com;
}

